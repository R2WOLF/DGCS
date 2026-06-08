"""Quick hardware-acceleration / backend checker for Raspberry Pi.

Runs a few probes to help determine why OpenCV isn't using the GPU for
decoding: prints OpenCV build info, queries ffmpeg/gstreamer for available
decoders/hwaccels, and attempts to open the provided RTSP URL with common
backends/pipelines.

Run on the Pi in the same environment used by your app.
"""
from __future__ import annotations

import subprocess
import sys
import shutil
import argparse

try:
    import cv2
except Exception:
    cv2 = None


def run_cmd(cmd: list[str]) -> tuple[int, str, str]:
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except FileNotFoundError:
        return 127, "", "not found"


def print_heading(h: str) -> None:
    print("\n=== ", h, " ===\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check OpenCV / ffmpeg / gstreamer hw-accel support")
    parser.add_argument("--url", required=True, help="RTSP URL to test (e.g. rtsp://CAM:8554/main.264)")
    args = parser.parse_args()

    print_heading("OpenCV build information")
    if cv2 is None:
        print("cv2 not importable in this Python environment")
    else:
        print(cv2.getBuildInformation())

    print_heading("ffmpeg availability and hwaccels")
    rc, out, err = run_cmd(["ffmpeg", "-hwaccels"]) if shutil.which("ffmpeg") else (127, "", "ffmpeg not found")
    if rc == 127:
        print("ffmpeg not found on PATH")
    else:
        print(out or err)

    print_heading("ffmpeg h264 decoders (if ffmpeg present)")
    if shutil.which("ffmpeg"):
        rc, out, err = run_cmd(["ffmpeg", "-decoders"]) 
        print('\n'.join([l for l in (out or err).splitlines() if 'h264' in l.lower()][:40]))
    else:
        print("ffmpeg missing")

    print_heading("gst-inspect (gstreamer) h264 elements")
    if shutil.which("gst-inspect-1.0"):
        rc, out, err = run_cmd(["gst-inspect-1.0"]) 
        lines = [l for l in (out or err).splitlines() if 'h264' in l.lower()]
        for l in lines[:200]:
            print(l)
        if not lines:
            print("No h264-related elements found in gst-inspect output")
    else:
        print("gst-inspect-1.0 not found")

    print_heading("Try opening with OpenCV backends")
    backends_to_try = []
    if cv2 is not None:
        # prefer constants when present
        if hasattr(cv2, "CAP_GSTREAMER"):
            backends_to_try.append(("GSTREAMER", cv2.CAP_GSTREAMER))
        if hasattr(cv2, "CAP_FFMPEG"):
            backends_to_try.append(("FFMPEG", cv2.CAP_FFMPEG))
        backends_to_try.append(("DEFAULT", None))

    for label, backend in backends_to_try:
        try:
            if backend is None:
                cap = cv2.VideoCapture(args.url)
            else:
                cap = cv2.VideoCapture(args.url, backend)
        except Exception as exc:
            print(f"{label}: VideoCapture() raised: {exc}")
            continue
        opened = cap.isOpened() if cap is not None else False
        print(f"{label}: opened={opened}")
        if opened:
            # try read a single frame
            ok, frame = cap.read()
            print(f"{label}: read ok={ok}, frame type={type(frame)}")
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

    print_heading("GStreamer candidate pipelines (manual test)")
    print("If you have GStreamer and hardware decoders, try one of these with cv2.CAP_GSTREAMER or gst-launch-1.0:")
    print("\nExample pipeline using v4l2 hardware decode (replace {URL}):")
    print("rtspsrc location={URL} latency=200 ! rtph264depay ! h264parse ! v4l2h264dec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true")
    print("\nOr try: omxh264dec or v4l2m2m depending on your Pi and installed plugins")

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
