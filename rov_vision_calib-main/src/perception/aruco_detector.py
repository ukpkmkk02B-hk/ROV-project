from __future__ import annotations

import cv2


def get_dictionary(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


class ArucoDetector:
    def __init__(self, dictionary_name: str, corner_refinement: bool = True):
        dictionary = get_dictionary(dictionary_name)
        params = cv2.aruco.DetectorParameters()
        if corner_refinement:
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

    def detect(self, frame):
        corners, ids, rejected = self.detector.detectMarkers(frame)
        return corners, ids, rejected
