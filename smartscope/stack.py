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
    data = array.astype(np.float32)

    if data.ndim == 3 and data.shape[2] >= 3:
        rgb = data[:, :, :3]
        if stretch:
            out = np.zeros_like(rgb, dtype=np.uint8)
            for c in range(3):
                channel = rgb[:, :, c]
                p_low, p_high = np.percentile(channel, [0.5, 99.5])
                if p_high > p_low:
                    out[:, :, c] = np.clip(
                        (channel - p_low) / (p_high - p_low) * 255.0, 0, 255
                    ).astype(np.uint8)
            data_u8 = out
        else:
            data_u8 = np.clip(rgb, 0, 255).astype(np.uint8)
        img = Image.fromarray(data_u8, mode="RGB")
    else:
        gray = data if data.ndim == 2 else np.squeeze(data).astype(np.float32)
        if stretch:
            p_low, p_high = np.percentile(gray, [0.5, 99.5])
            if p_high > p_low:
                data_u8 = np.clip(
                    (gray - p_low) / (p_high - p_low) * 255.0, 0, 255
                ).astype(np.uint8)
            else:
                data_u8 = np.zeros_like(gray, dtype=np.uint8)
        else:
            data_u8 = np.clip(gray, 0, 255).astype(np.uint8)
        img = Image.fromarray(data_u8, mode="L")

    img = img.resize(size, Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=75)
    return buf.getvalue()


def empty_preview_jpeg() -> bytes:
    """Return a 1x1 black JPEG placeholder."""
    buf = io.BytesIO()
    Image.new("RGB", (1, 1), (0, 0, 0)).save(buf, format="JPEG")
    return buf.getvalue()


class ImageStack:
    def __init__(self) -> None:
        self._sum: Optional[np.ndarray] = None
        self._count: int = 0

    def add_frame(self, array: np.ndarray) -> None:
        """Accumulate a new frame (RGB or grayscale uint8/float32)."""
        data = array.astype(np.float32)
        if data.ndim == 3 and data.shape[2] >= 3:
            frame = data[:, :, :3]
        else:
            frame = data

        if self._sum is None:
            self._sum = frame.copy()
        else:
            self._sum += frame
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
