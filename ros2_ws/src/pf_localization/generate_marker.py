#!/usr/bin/env python3
from pathlib import Path
import cv2
OUTPUT = Path(__file__).resolve().parent / 'materials' / 'textures' / 'aruco_marker.png'
MARKER_ID = 0
MARKER_SIZE_PX = 200

def main() -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        image = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, MARKER_SIZE_PX)
    else:
        image = cv2.aruco.drawMarker(dictionary, MARKER_ID, MARKER_SIZE_PX)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUTPUT), image)
    print(f'Wrote {OUTPUT} ({MARKER_SIZE_PX}x{MARKER_SIZE_PX}, ID={MARKER_ID})')
if __name__ == '__main__':
    main()
