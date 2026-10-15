"""Barcode and QR label decoding simulation with confidence and symbologies."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from drone_nav.world.models import Bin


@dataclass
class ScanReadResult:
    """Outcome of attempting to decode a bin label from camera capture."""

    success: bool
    read_type: str  # "BARCODE" | "QR" | "RFID" | "IMAGE_COUNT"
    symbology: str  # "CODE128" | "QRCODE"
    value: str | None
    confidence: float
    quality_score: float
    image_ref: str | None
    bbox: dict[str, int]
    failure_reason: str | None = None


class LabelReader:
    """Simulates computer vision / barcode engine decoding of bin labels."""

    def __init__(self, seed: int = 42, image_repo_path: Path | str = "Amazonbindata/img") -> None:
        self.rng = np.random.RandomState(seed)
        self.repo_path = Path(image_repo_path)
        self.available_images: list[str] = []
        if self.repo_path.exists() and self.repo_path.is_dir():
            # Gather up to 200 real images for realistic referencing
            found = list(self.repo_path.glob("*.jpg"))[:200]
            self.available_images = [p.name for p in found]

    def decode_label(
        self,
        target_bin: Bin,
        quality_score: float,
        bin_index: int = 0,
    ) -> ScanReadResult:
        """Attempt to decode label barcode/QR on target bin given image optical quality."""
        # Baseline bounding box in 1080p frame
        bbox = {
            "x": 860,
            "y": 490,
            "w": 200,
            "h": 100,
        }

        # Resolve image reference: prefer real paired image from catalog
        image_ref: str | None = None
        if target_bin.amazon_img_id:
            # Bin has a real paired Amazon image - store just the img_id for URL construction
            image_ref = target_bin.amazon_img_id
        elif self.available_images:
            img_name = self.available_images[bin_index % len(self.available_images)]
            # Store just the image filename (without repo path) for URL construction
            image_ref = img_name
        else:
            image_ref = target_bin.id

        # Label symbology: QR for higher density, CODE128 standard
        symbology = "CODE128" if (bin_index % 3 != 0) else "QRCODE"
        read_type = "BARCODE" if symbology == "CODE128" else "QR"

        # Quality threshold: below 0.20 quality is too blurry or far to read
        if quality_score < 0.20:
            return ScanReadResult(
                success=False,
                read_type=read_type,
                symbology=symbology,
                value=None,
                confidence=0.0,
                quality_score=quality_score,
                image_ref=image_ref,
                bbox=bbox,
                failure_reason="INSUFFICIENT_OPTICAL_QUALITY",
            )

        # Success probability model scaling with image quality Q
        # Q = 1.0 -> 98% decode probability
        # Q = 0.5 -> 85% decode probability
        # Q = 0.25 -> 50% decode probability
        p_success = min(0.99, max(0.0, 0.98 * (quality_score**0.4)))
        did_decode = bool(self.rng.uniform(0.0, 1.0) < p_success)

        if not did_decode:
            return ScanReadResult(
                success=False,
                read_type=read_type,
                symbology=symbology,
                value=None,
                confidence=round(float(max(0.1, 0.45 * quality_score)), 2),
                quality_score=quality_score,
                image_ref=image_ref,
                bbox=bbox,
                failure_reason="DECODE_PARSER_CRC_FAIL",
            )

        # Confidence score scaling with quality
        noise = float(self.rng.normal(0.0, 0.015))
        confidence = round(float(np.clip(0.72 + 0.26 * quality_score + noise, 0.60, 0.99)), 2)

        return ScanReadResult(
            success=True,
            read_type=read_type,
            symbology=symbology,
            value=target_bin.label_code,
            confidence=confidence,
            quality_score=quality_score,
            image_ref=image_ref,
            bbox=bbox,
            failure_reason=None,
        )
