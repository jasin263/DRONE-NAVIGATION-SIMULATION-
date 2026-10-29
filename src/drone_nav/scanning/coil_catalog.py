"""Client Steel Coil and Warehouse Image Catalog.

Provides structured metadata, mill supplier details, barcodes, and real image
mappings from the client's high-resolution warehouse photo dataset.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SteelCoilRecord:
    """Detailed metadata for a real warehouse coil or blank inventory item."""

    img_id: str  # e.g. "IMG_5805"
    coil_id: str  # e.g. "M30953188"
    barcode: str  # e.g. "303356989" or "PK20157854-15"
    mill_supplier: str  # ArcelorMittal, Cleveland-Cliffs, USS, PRO-TEC
    form_factor: str  # Master Coil, Slit Coil, Contoured Blank, Sheet Pack
    finish_grade: str  # Galvanized, Cold-Rolled, Hot-Rolled Pickled, Spangle
    approx_weight_kg: float
    floor_bay: str  # e.g. "R75", "R76-R77"
    column_landmark: str  # e.g. "B12", "D13", "C14"
    image_filename: str
    description: str


CLIENT_COIL_RECORDS: list[SteelCoilRecord] = [
    SteelCoilRecord(
        img_id="IMG_5798",
        coil_id="P3352-18",
        barcode="303356976",
        mill_supplier="Precision Slitting & Processing",
        form_factor="Sheet Pack",
        finish_grade="Cold-Rolled Sheet Stock",
        approx_weight_kg=4850.0,
        floor_bay="R71",
        column_landmark="D12",
        image_filename="IMG_5798.jpg",
        description="Stacked aluminum/steel sheet packs on wooden pallets, plastic wrapped with strapping.",
    ),
    SteelCoilRecord(
        img_id="IMG_5800",
        coil_id="8309350133",
        barcode="303356890",
        mill_supplier="Mill Coil Processing",
        form_factor="Strapped Sheet Pack",
        finish_grade="Cold-Rolled Steel",
        approx_weight_kg=5120.0,
        floor_bay="R72",
        column_landmark="D12",
        image_filename="IMG_5800.jpg",
        description="Strapped steel sheets with Code 128 barcode labels (303356890 / 8309350133).",
    ),
    SteelCoilRecord(
        img_id="IMG_5801",
        coil_id="M30934339-S3",
        barcode="62291000",
        mill_supplier="ArcelorMittal Calvert",
        form_factor="Master Cold-Rolled Coil",
        finish_grade="Cold-Rolled Commercial Steel",
        approx_weight_kg=18400.0,
        floor_bay="R73",
        column_landmark="D13",
        image_filename="IMG_5801.jpg",
        description="Master cold-rolled coil resting on rubber saddles along main aisle yellow boundary.",
    ),
    SteelCoilRecord(
        img_id="IMG_5802",
        coil_id="M30950797-S4",
        barcode="303356986",
        mill_supplier="PRO-TEC Coating Company",
        form_factor="Slit Steel Coil",
        finish_grade="Hot-Dip Galvanized CS Type B",
        approx_weight_kg=8650.0,
        floor_bay="R74",
        column_landmark="C12",
        image_filename="IMG_5802.jpg",
        description="Slit coils banded with pouch tag 303356986 / 74352-65A2A on floor skids.",
    ),
    SteelCoilRecord(
        img_id="IMG_5803",
        coil_id="M3094841",
        barcode="A0148NCA",
        mill_supplier="Cleveland-Cliffs",
        form_factor="Master Hot-Rolled Coil",
        finish_grade="Hot-Rolled High-Strength Steel",
        approx_weight_kg=24300.0,
        floor_bay="R74",
        column_landmark="D13",
        image_filename="IMG_5803.jpg",
        description="Large diameter master steel coils aligned along column D13 under gantry rails.",
    ),
    SteelCoilRecord(
        img_id="IMG_5805",
        coil_id="M30953188",
        barcode="30953188",
        mill_supplier="ArcelorMittal Calvert",
        form_factor="Master Galvanized Coil",
        finish_grade="Galvanized Automotive Grade",
        approx_weight_kg=21800.0,
        floor_bay="R75",
        column_landmark="B12",
        image_filename="IMG_5805.jpg",
        description="Master coil wrapped in branded ArcelorMittal protective shroud at stenciled bay R75.",
    ),
    SteelCoilRecord(
        img_id="IMG_5806",
        coil_id="M30954649CC",
        barcode="30954649",
        mill_supplier="PRO-TEC Coating Company",
        form_factor="Master Galvanized Coil",
        finish_grade="Galvannealed Steel",
        approx_weight_kg=19750.0,
        floor_bay="R75",
        column_landmark="B12",
        image_filename="IMG_5806.jpg",
        description="PRO-TEC branded master coil adjacent to overhead yellow inspection catwalk staircase.",
    ),
    SteelCoilRecord(
        img_id="IMG_5807",
        coil_id="PK20157854-15",
        barcode="PK20157854-15",
        mill_supplier="ArcelorMittal Calvert",
        form_factor="Master Coil with Core Protector",
        finish_grade="Structural Steel Grade B-30-B",
        approx_weight_kg=22600.0,
        floor_bay="R76",
        column_landmark="B13",
        image_filename="IMG_5807.jpg",
        description="Radial pleat wrapped coil with inner core protector sleeve and grade label B-30-B.",
    ),
    SteelCoilRecord(
        img_id="IMG_5808",
        coil_id="M30953502",
        barcode="GE660184200",
        mill_supplier="Cleveland-Cliffs",
        form_factor="Cold-Rolled Master Coil",
        finish_grade="Cold-Rolled Annealed Steel",
        approx_weight_kg=17900.0,
        floor_bay="R76",
        column_landmark="B13",
        image_filename="IMG_5808.jpg",
        description="Bare steel coil on heavy timber chocks with barcode tag GE660184200.",
    ),
    SteelCoilRecord(
        img_id="IMG_5809",
        coil_id="M30956124",
        barcode="30956124",
        mill_supplier="Cleveland-Cliffs",
        form_factor="Master Cold-Rolled Coil",
        finish_grade="High-Strength Low-Alloy (HSLA)",
        approx_weight_kg=23100.0,
        floor_bay="R77",
        column_landmark="E10",
        image_filename="IMG_5809.jpg",
        description="Cleveland-Cliffs wrapped master coil with marker 40-50 orientation arrow.",
    ),
    SteelCoilRecord(
        img_id="IMG_5810",
        coil_id="G62215",
        barcode="C64300",
        mill_supplier="United States Steel (USS)",
        form_factor="Cold-Rolled Slit Coil",
        finish_grade="Commercial Steel Type A",
        approx_weight_kg=11200.0,
        floor_bay="R74",
        column_landmark="B11",
        image_filename="IMG_5810.jpg",
        description="Aisle floor bay boundary R74-R75 with yellow barcode label C64300 / G62215.",
    ),
    SteelCoilRecord(
        img_id="IMG_5811",
        coil_id="M309344085",
        barcode="309344085",
        mill_supplier="United States Steel (USS)",
        form_factor="Dual Master Coil Row",
        finish_grade="Galvanized & Cold-Rolled",
        approx_weight_kg=20400.0,
        floor_bay="R75",
        column_landmark="B11",
        image_filename="IMG_5811.jpg",
        description="Corridor between USS coils and ArcelorMittal coils facing column B11/B12 stairs.",
    ),
    SteelCoilRecord(
        img_id="IMG_5813",
        coil_id="M309344085-USS",
        barcode="309344085",
        mill_supplier="United States Steel (USS)",
        form_factor="Master Coil",
        finish_grade="Ultra-High Strength Steel",
        approx_weight_kg=25600.0,
        floor_bay="R75",
        column_landmark="B10",
        image_filename="IMG_5813.jpg",
        description="USS protective wrap on high-gloss epoxy floor alongside heavy transport carrier.",
    ),
    SteelCoilRecord(
        img_id="IMG_5814",
        coil_id="WH-CRANE-BAY",
        barcode="BAY-C-CRANE",
        mill_supplier="Facility Logistics Bay",
        form_factor="Overhead 30-Ton Crane Runway",
        finish_grade="Industrial Facility",
        approx_weight_kg=30000.0,
        floor_bay="R70-R80",
        column_landmark="B10-B14",
        image_filename="IMG_5814.jpg",
        description="Full bay panoramic view of yellow overhead bridge crane spanning coil storage rows.",
    ),
    SteelCoilRecord(
        img_id="IMG_5815",
        coil_id="WH-RAIL-TRANSFER",
        barcode="TRACK-CAR-01",
        mill_supplier="Facility Logistics Bay",
        form_factor="Floor Rail System",
        finish_grade="Dual Transfer Tracks",
        approx_weight_kg=15000.0,
        floor_bay="TRACK-01",
        column_landmark="B13-B14",
        image_filename="IMG_5815.jpg",
        description="Dual embedded floor transfer rails for heavy automated coil cars.",
    ),
    SteelCoilRecord(
        img_id="IMG_5816",
        coil_id="4104408780",
        barcode="4104408780",
        mill_supplier="ArcelorMittal Calvert",
        form_factor="Master Galvanized Coil",
        finish_grade="Automotive Hot-Dip Galvanized",
        approx_weight_kg=21400.0,
        floor_bay="R76",
        column_landmark="C13",
        image_filename="IMG_5816.jpg",
        description="ArcelorMittal master coil with barcode 4104408780 and packaging ticket.",
    ),
    SteelCoilRecord(
        img_id="IMG_5817",
        coil_id="CONT-BLANK-AUTO",
        barcode="BLANK-74352",
        mill_supplier="Precision Blanking Facility",
        form_factor="Contoured Sheet Blanks",
        finish_grade="Automotive Body Panel Steel",
        approx_weight_kg=3900.0,
        floor_bay="R80",
        column_landmark="C09-C10",
        image_filename="IMG_5817.jpg",
        description="Laser-cut / stamped contoured body panel blanks stacked on heavy wooden skids.",
    ),
    SteelCoilRecord(
        img_id="IMG_5818",
        coil_id="WH-TRIPLE-CRANE",
        barcode="MULTI-BAY-CRANES",
        mill_supplier="Facility Logistics Bay",
        form_factor="Triple Crane Bays",
        finish_grade="Industrial Facility",
        approx_weight_kg=90000.0,
        floor_bay="ALL-BAYS",
        column_landmark="C12-C14",
        image_filename="IMG_5818.jpg",
        description="Three parallel overhead cranes with safety guardrails and multiple coil rows.",
    ),
    SteelCoilRecord(
        img_id="IMG_5819",
        coil_id="4083-GALV",
        barcode="4083",
        mill_supplier="ArcelorMittal / Cliffs",
        form_factor="Galvanized Master Coil",
        finish_grade="Galvanized Spangle Finish",
        approx_weight_kg=22100.0,
        floor_bay="R75",
        column_landmark="C14-C15",
        image_filename="IMG_5819.jpg",
        description="Galvanized master coil with handwritten 4083 arrow marking in front of gantry.",
    ),
]


class SteelCoilCatalog:
    """In-memory catalog and lookup service for client warehouse steel coils."""

    def __init__(self, records: list[SteelCoilRecord] | None = None) -> None:
        self._records = records or list(CLIENT_COIL_RECORDS)
        self._by_img_id = {r.img_id: r for r in self._records}
        self._by_barcode = {r.barcode: r for r in self._records}
        self._by_coil_id = {r.coil_id: r for r in self._records}

    @property
    def is_available(self) -> bool:
        return len(self._records) > 0

    def __len__(self) -> int:
        return len(self._records)

    def all_records(self) -> list[SteelCoilRecord]:
        return list(self._records)

    def get_by_index(self, idx: int) -> SteelCoilRecord:
        return self._records[idx % len(self._records)]

    def get_by_img_id(self, img_id: str) -> SteelCoilRecord | None:
        return self._by_img_id.get(img_id)

    def get_by_barcode(self, barcode: str) -> SteelCoilRecord | None:
        return self._by_barcode.get(barcode)

    def summary(self) -> dict[str, Any]:
        suppliers = list({r.mill_supplier for r in self._records})
        return {
            "total_records": len(self._records),
            "suppliers": suppliers,
            "form_factors": list({r.form_factor for r in self._records}),
            "total_images": len(self._records),
        }
