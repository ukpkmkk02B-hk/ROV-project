class ConstantVelocityEKF:
    """Lightweight constant-velocity estimator for visual tracking state."""

    def __init__(self, max_lost_frames=10, min_dt=1e-3):
        self.max_lost_frames = int(max_lost_frames)
        self.min_dt = float(min_dt)
        self._state = None
        self._last_timestamp = None
        self._last_measurement = None
        self._last_measurement_timestamp = None
        self._lost_frames = 0

    def reset(self):
        self._state = None
        self._last_timestamp = None
        self._last_measurement = None
        self._last_measurement_timestamp = None
        self._lost_frames = 0

    def update(self, pose, timestamp):
        timestamp = float(timestamp)
        measured = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "z": float(pose["z"]),
            "yaw": float(pose["yaw"]),
        }

        if self._last_measurement is None or self._last_measurement_timestamp is None:
            self._state = {
                **measured,
                "vx": 0.0,
                "vy": 0.0,
                "vz": 0.0,
                "yaw_rate_deg_s": 0.0,
            }
        else:
            dt = max(timestamp - self._last_measurement_timestamp, self.min_dt)
            previous = self._last_measurement
            self._state = {
                **measured,
                "vx": (measured["x"] - previous["x"]) / dt,
                "vy": (measured["y"] - previous["y"]) / dt,
                "vz": (measured["z"] - previous["z"]) / dt,
                "yaw_rate_deg_s": (measured["yaw"] - previous["yaw"]) / dt,
            }

        self._last_timestamp = timestamp
        self._last_measurement = measured
        self._last_measurement_timestamp = timestamp
        self._lost_frames = 0
        return self._snapshot(status="tracking", timestamp=timestamp)

    def predict(self, timestamp):
        timestamp = float(timestamp)
        if self._state is None or self._last_timestamp is None:
            return {
                "status": "lost",
                "lost_frames": self.max_lost_frames,
                "timestamp": timestamp,
            }

        dt = max(timestamp - self._last_timestamp, self.min_dt)
        self._state["x"] += self._state["vx"] * dt
        self._state["y"] += self._state["vy"] * dt
        self._state["z"] += self._state["vz"] * dt
        self._state["yaw"] += self._state["yaw_rate_deg_s"] * dt
        self._last_timestamp = timestamp
        self._lost_frames += 1

        status = "lost" if self._lost_frames >= self.max_lost_frames else "predicted"
        return self._snapshot(status=status, timestamp=timestamp)

    def _snapshot(self, status, timestamp):
        snapshot = dict(self._state)
        snapshot["status"] = status
        snapshot["lost_frames"] = self._lost_frames
        snapshot["timestamp"] = float(timestamp)
        return snapshot
