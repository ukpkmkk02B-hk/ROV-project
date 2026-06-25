from dataclasses import dataclass


def _clamp(value, limit):
    if limit is None:
        return value
    limit = abs(float(limit))
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class PidResult:
    error: float
    p: float
    i: float
    d: float
    output: float
    integral: float
    dt: float


class PidAxisController:
    """Single-axis PID controller with symmetric integral and output limits."""

    def __init__(
        self,
        kp=0.0,
        ki=0.0,
        kd=0.0,
        integral_limit=None,
        output_limit=None,
        derivative_min_dt_s=0.0,
        d_limit=None,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.derivative_min_dt_s = float(derivative_min_dt_s or 0.0)
        self.d_limit = d_limit
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._last_error = None
        self._last_timestamp = None

    def update(self, error, timestamp=None):
        error = float(error)
        dt = self._compute_dt(timestamp)
        accept_reference = self._last_error is None or timestamp is None or self._last_timestamp is None
        if dt > 0.0 and dt >= self.derivative_min_dt_s:
            self._integral = _clamp(self._integral + error * dt, self.integral_limit)
            if self._last_error is None:
                derivative = 0.0
            else:
                derivative = (error - self._last_error) / dt
            accept_reference = True
        else:
            derivative = 0.0

        p = self.kp * error
        i = self.ki * self._integral
        d = _clamp(self.kd * derivative, self.d_limit)
        output = _clamp(p + i + d, self.output_limit)

        if accept_reference:
            self._last_error = error
            if timestamp is not None:
                self._last_timestamp = float(timestamp)

        return PidResult(error=error, p=p, i=i, d=d, output=output, integral=self._integral, dt=dt)

    def _compute_dt(self, timestamp):
        if timestamp is None or self._last_timestamp is None:
            return 0.0
        dt = float(timestamp) - self._last_timestamp
        return dt if dt > 0.0 else 0.0


class MultiAxisPidController:
    """Small wrapper for the four visual tracking PID axes."""

    def __init__(self, axis_configs):
        self.controllers = {
            axis: PidAxisController(
                kp=config.get("kp", 0.0),
                ki=config.get("ki", 0.0),
                kd=config.get("kd", 0.0),
                integral_limit=config.get("integral_limit"),
                output_limit=config.get("output_limit"),
                derivative_min_dt_s=config.get("derivative_min_dt_s", 0.0),
                d_limit=config.get("d_limit"),
            )
            for axis, config in (axis_configs or {}).items()
        }

    def reset(self):
        for controller in self.controllers.values():
            controller.reset()

    def update(self, errors, timestamp=None):
        outputs = {}
        diagnostics = {}
        for axis, error in (errors or {}).items():
            controller = self.controllers.get(axis)
            if controller is None:
                continue
            result = controller.update(error, timestamp=timestamp)
            outputs[axis] = result.output
            prefix = f"pid_{axis}"
            diagnostics[f"{prefix}_error"] = result.error
            diagnostics[f"{prefix}_p"] = result.p
            diagnostics[f"{prefix}_i"] = result.i
            diagnostics[f"{prefix}_d"] = result.d
            diagnostics[f"{prefix}_output"] = result.output
        return outputs, diagnostics
