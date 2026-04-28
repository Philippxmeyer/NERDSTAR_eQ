"""Running image stack with JPEG preview output."""

import io
from typing import Optional, Tuple

import numpy as np
from PIL import Image


PREVIEW_SIZE: Tuple[int, int] = (960, 540)


def render_preview_jpeg(
    array: np.ndarray,
    size: Tuple[int, int] = PREVIEW_SIZE,
    stretch: bool = True,
) -> bytes:
    """Render any 2D/3D array as a downsampled JPEG.

    Used both by the running stack and by the live finder preview so the
    web UI sees consistent dimensions and contrast.
    """
    if array.ndim == 3:
        gray = np.mean(array, axis=2).astype(np.float32)
    else:
        gray = array.astype(np.float32)

    if stretch:
        p_low, p_high = np.percentile(gray, [0.5, 99.5])
        if p_high > p_low:
            data = np.clip(
                (gray - p_low) / (p_high - p_low) * 255.0, 0, 255
            ).astype(np.uint8)
        else:
            data = np.zeros_like(gray, dtype=np.uint8)
    else:
        data = np.clip(gray, 0, 255).astype(np.uint8)

    img = Image.fromarray(data, mode="L")
    img = img.resize(size, Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=75)
    return buf.getvalue()


def empty_preview_jpeg() -> bytes:
    """Return a 1x1 black JPEG placeholder."""
    buf = io.BytesIO()
    Image.new("L", (1, 1), 0).save(buf, format="JPEG")
    return buf.getvalue()


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
            return empty_preview_jpeg()
        return render_preview_jpeg(self._sum / self._count)

    def reset(self) -> None:
        self._sum = None
        self._count = 0

    @property
    def frame_count(self) -> int:
        return self._count
