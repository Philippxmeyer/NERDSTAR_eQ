"""Running image stack with JPEG preview output."""

import io
from typing import Optional

import numpy as np
from PIL import Image


class ImageStack:
    def __init__(self) -> None:
        self._sum: Optional[np.ndarray] = None
        self._count: int = 0

    def add_frame(self, array: np.ndarray) -> None:
        """Accumulate a new frame (RGB or grayscale uint8/float32)."""
        if array.ndim == 3:
            gray = np.mean(array, axis=2).astype(np.float32)
        else:
            gray = array.astype(np.float32)

        if self._sum is None:
            self._sum = gray.copy()
        else:
            self._sum += gray
        self._count += 1

    def get_preview_jpeg(self) -> bytes:
        """Return a downsampled, contrast-stretched JPEG of the current stack."""
        if self._sum is None or self._count == 0:
            buf = io.BytesIO()
            Image.new("L", (1, 1), 0).save(buf, format="JPEG")
            return buf.getvalue()

        mean = self._sum / self._count

        # Percentile stretch to bring out faint detail
        p_low, p_high = np.percentile(mean, [0.5, 99.5])
        if p_high > p_low:
            stretched = np.clip(
                (mean - p_low) / (p_high - p_low) * 255.0, 0, 255
            ).astype(np.uint8)
        else:
            stretched = np.zeros_like(mean, dtype=np.uint8)

        img = Image.fromarray(stretched, mode="L")
        # Downsample to 960×540 for web preview
        img = img.resize((960, 540), Image.LANCZOS)

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=75)
        return buf.getvalue()

    def reset(self) -> None:
        self._sum = None
        self._count = 0

    @property
    def frame_count(self) -> int:
        return self._count
