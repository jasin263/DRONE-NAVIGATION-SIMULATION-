"""Amazon Bin Image Dataset catalog loader.

Reads the real Amazon Bin Image Dataset SQLite database (metadata.sqlite) and
provides fast, deterministic lookup of real bin records keyed by img_id.

Each record contains:
  - img_id       : matches the image filename stem (e.g. "00005" → "00005.jpg")
  - image_path   : relative path to the JPEG image
  - asins        : list of ASIN strings present in the bin
  - items        : list of BinItem dicts with keys:
                     asin, name, quantity, height_in, length_in, width_in, weight_lb
  - expected_qty : total expected quantity count in the bin
  - primary_asin : the ASIN with the highest quantity (used as label_code)
"""

from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from pathlib import Path


def _dim_value(node: object) -> float:
    """Extract the numeric ``value`` from a nested measurement dict, defensively."""
    if isinstance(node, dict):
        return float(node.get("value", 0.0) or 0.0)
    return 0.0


@dataclass
class BinItem:
    """One line-item inside an Amazon bin."""

    asin: str
    name: str
    quantity: int
    height_in: float
    length_in: float
    width_in: float
    weight_lb: float


@dataclass
class AmazonBinRecord:
    """Full record for a single Amazon warehouse bin image."""

    img_id: str          # e.g. "00005"
    image_path: str      # e.g. "Amazonbindata/img/00005.jpg"
    asins: list[str]
    items: list[BinItem]
    expected_qty: int
    primary_asin: str    # ASIN with max quantity – used as label_code

    @property
    def total_quantity(self) -> int:
        return sum(i.quantity for i in self.items)


class AmazonBinCatalog:
    """
    Loads the Amazon Bin Image Dataset from SQLite once, then exposes two
    access patterns:

        catalog.by_index(n)          → AmazonBinRecord (deterministic wrap)
        catalog.by_img_id("00005")   → AmazonBinRecord | None
        len(catalog)                 → total available records
    """

    def __init__(
        self,
        db_path: Path | str = "Amazonbindata/metadata.sqlite",
        img_dir: str = "Amazonbindata/img",
    ) -> None:
        self._db_path = Path(db_path)
        self._img_dir = img_dir
        self._records: list[AmazonBinRecord] = []
        self._by_id: dict[str, AmazonBinRecord] = {}
        self._loaded = False

        if self._db_path.exists():
            self._load()

    # ------------------------------------------------------------------
    # Internal loading
    # ------------------------------------------------------------------

    def _load(self) -> None:
        conn = sqlite3.connect(str(self._db_path))
        try:
            cur = conn.cursor()
            cur.execute("SELECT img_id, data FROM metadata")
            rows = cur.fetchall()
        finally:
            conn.close()

        for img_id, data_json in rows:
            try:
                data = json.loads(data_json)
            except (json.JSONDecodeError, TypeError):
                continue

            if not isinstance(data, dict):
                continue

            bin_fcsku = data.get("BIN_FCSKU_DATA") or {}
            if not isinstance(bin_fcsku, dict):
                bin_fcsku = {}
            expected_qty = int(data.get("EXPECTED_QUANTITY", 0) or 0)

            items: list[BinItem] = []
            for asin, info in bin_fcsku.items():
                if not isinstance(info, dict):
                    continue
                items.append(
                    BinItem(
                        asin=asin,
                        name=str(info.get("name", asin)),
                        quantity=int(info.get("quantity", 1) or 1),
                        height_in=_dim_value(info.get("height")),
                        length_in=_dim_value(info.get("length")),
                        width_in=_dim_value(info.get("width")),
                        weight_lb=_dim_value(info.get("weight")),
                    )
                )

            if not items:
                continue

            # Dominant ASIN = highest quantity (ties broken by ASIN string sort)
            primary = max(items, key=lambda it: (it.quantity, it.asin))

            img_filename = f"{img_id}.jpg"
            img_path = f"{self._img_dir}/{img_filename}"

            rec = AmazonBinRecord(
                img_id=img_id,
                image_path=img_path,
                asins=[it.asin for it in items],
                items=items,
                expected_qty=expected_qty,
                primary_asin=primary.asin,
            )
            self._records.append(rec)
            self._by_id[img_id] = rec

        self._loaded = True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_available(self) -> bool:
        """True if the database was found and loaded successfully."""
        return self._loaded and len(self._records) > 0

    def __len__(self) -> int:
        return len(self._records)

    def by_index(self, n: int) -> AmazonBinRecord | None:
        """Return record at index n (wraps around). Returns None if catalog empty."""
        if not self._records:
            return None
        return self._records[n % len(self._records)]

    def by_img_id(self, img_id: str) -> AmazonBinRecord | None:
        """Return record by exact img_id string, or None."""
        return self._by_id.get(img_id)

    def summary(self) -> dict:
        """High-level statistics about the loaded catalog."""
        if not self._records:
            return {"loaded": False, "records": 0}
        total_items = sum(len(r.items) for r in self._records)
        total_qty = sum(r.expected_qty for r in self._records)
        unique_asins: set[str] = set()
        for r in self._records:
            unique_asins.update(r.asins)
        return {
            "loaded": True,
            "records": len(self._records),
            "unique_asins": len(unique_asins),
            "total_line_items": total_items,
            "total_expected_qty": total_qty,
        }
