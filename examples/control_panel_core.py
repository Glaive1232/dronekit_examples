"""Telemetry, keyboard RC state and bounded DroneKit flight operations.

The RC mapping/pulse range is the standard ArduCopter SITL configuration.
This module imports neither DroneKit nor Tkinter, so it can be unit tested.
"""
from collections import deque
import math
import threading
import time


def validate_altitude(value):
    try:
        altitude = float(value)
    except (TypeError, ValueError):
        raise ValueError("Enter a positive, finite takeoff altitude in metres.") from None
    if isinstance(value, bool) or not math.isfinite(altitude) or altitude <= 0:
        raise ValueError("Enter a positive, finite takeoff altitude in metres.")
    return altitude


def pwm_value(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if isinstance(value, bool) or not math.isfinite(number) or not 800 <= number <= 2200:
        return None
    return round(number)


class HeartbeatMonitor:
    def __init__(self, clock=time.monotonic, timeout=5):
        self.clock = clock
        self.timeout = timeout
        self._received = None
        self._lock = threading.Lock()

    def observe(self, age):
        try:
            age = float(age)
        except (TypeError, ValueError):
            return
        if not math.isfinite(age) or age < 0:
            return
        with self._lock:
            self._received = self.clock() - age

    @property
    def age(self):
        with self._lock:
            received = self._received
        return None if received is None else max(0, self.clock() - received)

    @property
    def healthy(self):
        age = self.age
        return age is not None and age < self.timeout

    def reset(self):
        with self._lock:
            self._received = None


class KeyboardRC:
    """Held keys are a set: OS key-repeat never multiplies an input."""
    KEYS = frozenset(('w', 's', 'a', 'd', 'left', 'right', 'up', 'down'))

    def __init__(self):
        self.enabled = False
        self._held = set()
        self._throttle = 1000.0

    def enable(self, throttle):
        try:
            value = float(throttle)
        except (TypeError, ValueError):
            raise ValueError("A valid throttle signal is needed to enable keyboard RC.") from None
        if not math.isfinite(value):
            raise ValueError("A valid throttle signal is needed to enable keyboard RC.")
        self._held.clear()
        self._throttle = float(max(1000, min(2000, value)))
        self.enabled = True

    def disable(self):
        self.enabled = False
        self._held.clear()

    def press(self, key):
        key = key.lower()
        if self.enabled and key in self.KEYS:
            self._held.add(key)

    def release(self, key):
        self._held.discard(key.lower())

    def _axis(self, negative, positive):
        return 1500 + 200 * (int(positive in self._held) - int(negative in self._held))

    @property
    def values(self):
        if not self.enabled:
            return {}
        return {'1': self._axis('left', 'right'),
                '2': self._axis('up', 'down'),
                '3': round(self._throttle),
                '4': self._axis('a', 'd')}

    def update(self, dt):
        if self.enabled:
            dt = max(0.0, min(0.2, dt))
            direction = int('w' in self._held) - int('s' in self._held)
            self._throttle = max(1000.0, min(2000.0, self._throttle + direction * 200 * dt))
        return self.values


class OperationCancelled(RuntimeError):
    pass


class FlightController:
    MANUAL_MODES = frozenset(('STABILIZE', 'LOITER', 'ALT_HOLD', 'POSHOLD'))
    _COMMAND_LABELS = {400: 'Arm/disarm', 22: 'Takeoff', 176: 'Mode change'}
    _ACK_ERRORS = {1: 'temporarily rejected', 2: 'denied', 3: 'unsupported',
                   4: 'failed', 6: 'cancelled'}

    def __init__(self, clock=time.monotonic, sleep=time.sleep, mode_factory=None):
        self.clock = clock
        self.sleep = sleep
        self.mode_factory = mode_factory
        self.vehicle = None
        self.heartbeat = HeartbeatMonitor(clock)
        self._listener = None
        self._stream_listeners = []
        self._seen = {}
        self._feedback_lock = threading.Lock()
        self._session = None
        self._notice_sequence = 0
        self._notices = deque(maxlen=100)
        self._warnings = deque(maxlen=100)
        self._rejections = deque(maxlen=100)

    def _reset_feedback(self, session=None):
        with self._feedback_lock:
            self._session = session
            self._notices.clear()
            self._warnings.clear()
            self._rejections.clear()

    def drain_notices(self):
        """Return queued autopilot warnings/errors once, safely across threads."""
        with self._feedback_lock:
            notices = list(self._notices)
            self._notices.clear()
        return notices

    def _record_feedback(self, vehicle, session, name, message):
        with self._feedback_lock:
            if self.vehicle is not vehicle or self._session is not session:
                return
            if name == 'STATUSTEXT':
                try:
                    severity = int(message.severity)
                except (AttributeError, TypeError, ValueError, OverflowError):
                    return
                if not 0 <= severity <= 4:
                    return
                text = getattr(message, 'text', '')
                if isinstance(text, bytes):
                    text = text.decode('utf-8', errors='replace')
                if not isinstance(text, str):
                    return
                text = text.split('\0', 1)[0].strip()
                if not text:
                    return
                self._notice_sequence += 1
                self._warnings.append((self._notice_sequence, self.clock(), text))
            else:
                try:
                    command, result = int(message.command), int(message.result)
                except (AttributeError, TypeError, ValueError, OverflowError):
                    return
                if result not in self._ACK_ERRORS:
                    return
                label = self._COMMAND_LABELS.get(command, 'Command {}'.format(command))
                text = '{} {} by autopilot.'.format(label, self._ACK_ERRORS[result])
                self._notice_sequence += 1
                self._rejections.append((self._notice_sequence, command, text))
            self._notices.append(text)

    def _feedback_marker(self):
        with self._feedback_lock:
            return self._notice_sequence

    def _recent_reason(self, command, since=-1, max_age=30):
        with self._feedback_lock:
            return self._recent_reason_locked(command, since, max_age)

    def _recent_reason_locked(self, command, since, max_age):
        prefixes = {400: ('prearm:', 'prearm ', 'arm:', 'arming:'),
                    22: ('takeoff', 'take off'), 176: ('mode',)}.get(command, ())
        for sequence, received, text in reversed(self._warnings):
            if sequence > since and self.clock() - received <= max_age:
                if text.lower().startswith(prefixes):
                    return text
        return None

    def _command_rejection(self, command, since):
        with self._feedback_lock:
            for sequence, rejected_command, message in reversed(self._rejections):
                if sequence > since and rejected_command == command:
                    reason = self._recent_reason_locked(command, since, 5)
                    return message + (' ' + reason if reason else '')
        return None

    def attach(self, vehicle):
        if self.vehicle is not None:
            raise RuntimeError("Close the existing connection first.")
        self.heartbeat.reset()
        self.vehicle = vehicle
        session = object()
        self._reset_feedback(session)

        def listener(observed_vehicle, name, age):
            if self.vehicle is vehicle and self._session is session:
                self.heartbeat.observe(age)

        self._listener = listener
        self._seen = {'altitude': self.clock(), 'channels': self.clock()}
        try:
            vehicle.add_attribute_listener('last_heartbeat', listener)
            self.heartbeat.observe(getattr(vehicle, 'last_heartbeat', None))
            # Each sensor stream can stop independently of HEARTBEAT.
            if hasattr(vehicle, 'add_message_listener'):
                for message, stream in (('GLOBAL_POSITION_INT', 'altitude'),
                                        ('RC_CHANNELS', 'channels'),
                                        ('RC_CHANNELS_RAW', 'channels')):
                    def remember(observed_vehicle, name, value, stream=stream):
                        if self.vehicle is vehicle and self._session is session:
                            self._seen[stream] = self.clock()
                    vehicle.add_message_listener(message, remember)
                    self._stream_listeners.append((message, remember))
                for message in ('STATUSTEXT', 'COMMAND_ACK'):
                    def feedback(observed_vehicle, name, value):
                        self._record_feedback(vehicle, session, name, value)
                    vehicle.add_message_listener(message, feedback)
                    self._stream_listeners.append((message, feedback))
        except Exception:
            self.vehicle = None
            self.heartbeat.reset()
            self._reset_feedback()
            self._remove_listeners(vehicle)
            raise

    def _remove_listeners(self, vehicle):
        try:
            if self._listener is not None:
                vehicle.remove_attribute_listener('last_heartbeat', self._listener)
            for name, listener in self._stream_listeners:
                vehicle.remove_message_listener(name, listener)
        finally:
            self._listener = None
            self._stream_listeners = []
            self._seen = {}

    def _fresh(self, stream):
        received = self._seen.get(stream)
        return received is not None and self.clock() - received < self.heartbeat.timeout

    def _altitude(self, vehicle):
        if not self._fresh('altitude'):
            return None
        location = getattr(getattr(vehicle, 'location', None), 'global_relative_frame', None)
        try:
            value = float(getattr(location, 'alt', None))
            return value if math.isfinite(value) else None
        except (TypeError, ValueError):
            return None

    def disconnect(self):
        vehicle = self.vehicle
        self.vehicle = None
        self.heartbeat.reset()
        self._reset_feedback()
        if vehicle is None:
            return
        try:
            self._remove_listeners(vehicle)
            vehicle.channels.overrides = {}
        finally:
            self._listener = None
            vehicle.close()

    def snapshot(self):
        vehicle = self.vehicle
        result = {'attached': vehicle is not None, 'connected': False,
                  'heartbeat_age': self.heartbeat.age, 'mode': None, 'armed': None,
                  'altitude': None, 'channels': {str(i): None for i in range(1, 5)}}
        if vehicle is None:
            return result
        result['connected'] = self.heartbeat.healthy
        result['mode'] = getattr(getattr(vehicle, 'mode', None), 'name', None)
        result['armed'] = getattr(vehicle, 'armed', None)
        result['altitude'] = self._altitude(vehicle)
        channels = (getattr(vehicle, 'channels', None) or {}) if self._fresh('channels') else {}
        result['channels'] = {str(i): pwm_value(channels.get(str(i))) for i in range(1, 5)}
        return result

    def _check(self, vehicle, cancel):
        if cancel is not None and cancel.is_set():
            raise OperationCancelled("Operation cancelled.")
        if vehicle is None or self.vehicle is not vehicle or not self.heartbeat.healthy:
            raise RuntimeError("Connection lost: no recent vehicle heartbeat.")

    def _wait(self, predicate, vehicle, cancel, timeout, description,
              command=None, since=None):
        deadline = self.clock() + timeout
        while True:
            self._check(vehicle, cancel)
            if command is not None:
                rejection = self._command_rejection(command, since)
                if rejection:
                    raise RuntimeError(rejection)
            if predicate():
                return
            if self.clock() >= deadline:
                reason = self._recent_reason(command, since) if command is not None else None
                message = description + " was not confirmed before timeout."
                raise TimeoutError(message + (' ' + reason if reason else ''))
            self.sleep(0.1)

    def _mode(self, name):
        if self.mode_factory is None:
            from dronekit import VehicleMode
            return VehicleMode(name)
        return self.mode_factory(name)

    def validate_mode_change(self, name):
        """Validate a mode request before the caller changes mode or RC inputs."""
        vehicle = self.vehicle
        self._check(vehicle, None)
        current = getattr(getattr(vehicle, 'mode', None), 'name', None)
        if name.upper() == 'STABILIZE' and current != 'STABILIZE' and vehicle.armed:
            altitude = self._altitude(vehicle)
            if altitude is None or altitude > 0.5:
                raise RuntimeError('Stabilize has no altitude hold. Use Loiter after takeoff; '
                                   'start Stabilize manually from ground.')

    def change_mode(self, name, cancel=None):
        vehicle = self.vehicle
        self._check(vehicle, cancel)
        self.validate_mode_change(name)
        marker = self._feedback_marker()
        vehicle.mode = self._mode(name.upper())
        self._wait(lambda: vehicle.mode.name == name.upper(), vehicle, cancel, 15, name,
                   command=176, since=marker)

    def arm(self, cancel=None):
        vehicle = self.vehicle
        self._check(vehicle, cancel)
        if vehicle.armed:
            return
        mode = getattr(getattr(vehicle, 'mode', None), 'name', None)
        if mode in ('LAND', 'RTL'):
            raise RuntimeError('Cannot arm in {} mode. Choose Guided, Stabilize or Loiter first.'
                               .format(mode))
        if not vehicle.is_armable:
            reason = self._recent_reason(400)
            raise RuntimeError(reason or "Vehicle is not armable. Check the simulator's pre-arm status.")
        marker = self._feedback_marker()
        vehicle.armed = True
        self._wait(lambda: vehicle.armed, vehicle, cancel, 15, "Arming",
                   command=400, since=marker)

    def takeoff(self, altitude, cancel=None, progress=None):
        target = validate_altitude(altitude)
        vehicle = self.vehicle
        self._check(vehicle, cancel)
        if not vehicle.armed and not vehicle.is_armable:
            reason = self._recent_reason(400)
            raise RuntimeError(reason or "Vehicle is not armable. Wait for pre-arm checks to pass.")
        notify = progress or (lambda message: None)
        notify("Switching to Guided…")
        self.change_mode('GUIDED', cancel)
        notify("Arming for takeoff…")
        self.arm(cancel)
        self._check(vehicle, cancel)
        if vehicle.mode.name != 'GUIDED':
            raise OperationCancelled("Takeoff stopped: flight mode changed.")
        marker = self._feedback_marker()
        vehicle.simple_takeoff(target)
        notify("Climbing to {:.1f} m…".format(target))

        def reached():
            if not vehicle.armed:
                raise RuntimeError("Takeoff stopped: vehicle disarmed.")
            if vehicle.mode.name != 'GUIDED':
                raise OperationCancelled("Takeoff stopped: flight mode changed.")
            value = self._altitude(vehicle)
            return value is not None and value >= target * 0.95

        self._wait(reached, vehicle, cancel, 120, "Takeoff altitude",
                   command=22, since=marker)
        return target
