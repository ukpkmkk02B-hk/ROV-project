#!/usr/bin/env python3
"""Diagnose which dictionary matches the user's printed tag.
Samples frames and prints the best matching dictionary to stdout.
"""
import sys
import time
import cv2
import numpy as np

# All common dictionaries to try
DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_ARUCO_MIP_36h12": cv2.aruco.DICT_ARUCO_MIP_36h12,
}


def main():
    camera_device = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(camera_device)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera /dev/video{camera_device}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera opened: {w}x{h}")

    total_frames = 60  # sample more frames for better detection
    print(f"\nSampling {total_frames} frames to auto-detect marker type...")
    print("Hold the marker still at 20-50cm from camera.\n")
    print("Press 'q' or ESC to quit early.\n")

    # Create window
    cv2.namedWindow("Tag Diagnosis - Press q to quit", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Tag Diagnosis - Press q to quit", 960, 540)

    # Accumulate results from all frames
    all_results = {}  # name -> set of ids seen
    frame_any_detections = set()  # dictionaries that ever detected anything this frame

    i = 0
    while i < total_frames:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed, retrying...")
            time.sleep(0.1)
            continue

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_any_detections.clear()

        for name, dict_id in DICTS.items():
            dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            detector = cv2.aruco.ArucoDetector(dictionary)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                flat = [int(x) for x in ids.flatten()]
                all_results.setdefault(name, set()).update(flat)
                frame_any_detections.add(name)
                # Draw detected markers on display
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

        # Draw overlay info
        info_lines = [f"Frame: {i+1}/{total_frames}"]
        if frame_any_detections:
            info_lines.append(f"Detected: {len(frame_any_detections)} dict(s)")
            # Show top dict so far
            sorted_sofar = sorted(all_results.items(), key=lambda x: len(x[1]), reverse=True)
            best_name, best_ids = sorted_sofar[0]
            info_lines.append(f"BEST: {best_name}")
            info_lines.append(f"  IDs: {sorted(best_ids)}")
        else:
            info_lines.append("NO DETECTION - show your tag!")

        y0 = 30
        for line in info_lines:
            cv2.putText(display, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)
            y0 += 30

        cv2.imshow("Tag Diagnosis - Press q to quit", display)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q') or key == 27:  # q or ESC
            print("\nUser quit early.\n")
            break

        if (i + 1) % 10 == 0:
            print(f"  ... sampled {i+1}/{total_frames} frames")

        i += 1
        time.sleep(0.03)

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)

    if not all_results:
        print("\n❌ NO MATCHES FOUND AT ALL")
        print("\nPossible reasons:")
        print("  1. The printed tag is NOT an ArUco or AprilTag marker")
        print("  2. The tag is too far / too close / out of focus")
        print("  3. The tag is heavily distorted or printed too small")
        print("  4. Lighting is too poor")
        print("\nTry: bring the tag to 20cm from camera with good light.")
        return

    # Sort by most IDs found (best dictionary)
    sorted_results = sorted(all_results.items(), key=lambda x: len(x[1]), reverse=True)

    print(f"\n✅ Detected {len(sorted_results)} matching dictionaries:\n")
    for name, ids_set in sorted_results:
        ids_list = sorted(ids_set)
        print(f"  🟢 {name:<28}  IDs: {ids_list}  ({len(ids_list)} distinct)")

    # Find the dictionary with the MOST CONSISTENT single ID (fewest distinct IDs = best match)
    # A good dictionary detects exactly 1 ID across all frames
    best = sorted_results[0]
    # Prefer dict with fewer distinct IDs (more consistent)
    sorted_by_consistency = sorted(sorted_results, key=lambda x: (len(x[1]), -len(x[1])))
    # Actually: prefer dict that detected the most frames with a SINGLE ID
    best = min(sorted_results, key=lambda x: len(x[1]))  # fewest distinct IDs
    # But also ensure it actually detected a reasonable amount
    # Fallback: pick the one with most distinct IDs if none have just 1
    if len(best[1]) > 3:
        best = sorted_results[0]  # fallback to most-active dict

    print(f"\n🏆 BEST MATCH: {best[0]}  IDs: {sorted(best[1])}")
    print(f"\nUse this in config/aruco_config.yaml:")
    print(f"  dictionary: \"{best[0]}\"")
    print(f"  marker_id: {sorted(best[1])[0]}  # or whichever ID you printed")


if __name__ == "__main__":
    main()
