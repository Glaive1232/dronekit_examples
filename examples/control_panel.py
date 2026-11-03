"""Tkinter DroneKit panel: telemetry, keyboard RC and guided takeoff.

Run with Python 3.9 and DroneKit in the project's SITL container.
"""
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from control_panel_core import FlightController, KeyboardRC, OperationCancelled, validate_altitude


class ControlPanel:
    MODES = ('Guided', 'Stabilize', 'Loiter', 'Land', 'RTL')
    CHANNELS = (('1', 'Roll', 'Left / Right'), ('2', 'Pitch', 'Up / Down'),
                ('3', 'Throttle', 'W / S'), ('4', 'Yaw', 'A / D'))

    def __init__(self, root, controller=None):
        self.root = root
        self.controller = controller or FlightController()
        self.rc = KeyboardRC()
        self.busy = False
        self.closing = False
        self._destroyed = False
        self._operation = None
        self._cancel = None
        self._pending = None
        self._job_id = 0
        self._poll_job = None
        self._keyups = {}
        self._rc_owned = False
        self._hold_values = None
        self._hold_mode = None
        self._keyboard_mode = None
        self._last_tick = time.monotonic()
        self.events = queue.Queue()
        root.title('DroneKit Control Panel')
        root.resizable(True, True)
        root.minsize(840, 560)
        root.geometry('900x620')
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        frame = ttk.Frame(root, padding=10)
        frame.grid(row=0, column=0, sticky='nsew')
        for column in range(3):
            frame.columnconfigure(column, weight=1)

        connection_box = ttk.LabelFrame(frame, text='Connection', padding=6)
        connection_box.grid(row=0, column=0, columnspan=3, sticky='ew', pady=(0, 6))
        connection_box.columnconfigure(0, weight=1)
        self.connection = tk.StringVar(value='udpin:0.0.0.0:14450')
        self.entry = ttk.Entry(connection_box, textvariable=self.connection, width=36)
        self.entry.grid(row=0, column=0, sticky='ew', padx=(0, 8))
        self.connect_button = ttk.Button(connection_box, text='Connect', command=self.connect)
        self.connect_button.grid(row=0, column=1)
        self.close_button = ttk.Button(connection_box, text='Close Connection', command=self.disconnect)
        self.close_button.grid(row=0, column=2, padx=(8, 0))

        state_box = ttk.LabelFrame(frame, text='Current state', padding=6)
        state_box.grid(row=1, column=0, columnspan=3, sticky='ew', pady=(0, 6))
        self.connection_status = tk.StringVar(value='Disconnected')
        self.mode_text = tk.StringVar(value='—')
        self.armed_text = tk.StringVar(value='—')
        self.altitude_text = tk.StringVar(value='—')
        self.heartbeat_text = tk.StringVar(value='—')
        for column, (label, variable) in enumerate((('Connection', self.connection_status),
                ('Mode', self.mode_text), ('Motors', self.armed_text),
                ('Altitude (relative)', self.altitude_text), ('Last heartbeat', self.heartbeat_text))):
            state_box.columnconfigure(column, weight=1)
            ttk.Label(state_box, text=label).grid(row=0, column=column, sticky='w', padx=(0, 12))
            ttk.Label(state_box, textvariable=variable, font=('TkDefaultFont', 10, 'bold')).grid(
                row=1, column=column, sticky='w', padx=(0, 12))

        mode_box = ttk.LabelFrame(frame, text='Flight controls', padding=6)
        mode_box.grid(row=2, column=0, columnspan=3, sticky='ew', pady=(0, 6))
        self.mode_buttons = []
        for index, mode in enumerate(self.MODES):
            button = ttk.Button(mode_box, text=mode, command=lambda name=mode: self.change_mode(name))
            button.grid(row=0, column=index, padx=3, sticky='ew')
            mode_box.columnconfigure(index, weight=1)
            self.mode_buttons.append(button)
        self.arm_button = ttk.Button(mode_box, text='Arm', command=self.arm)
        self.arm_button.grid(row=1, column=0, padx=3, pady=(6, 0), sticky='ew')
        ttk.Label(mode_box, text='Takeoff altitude (m):').grid(row=1, column=1, columnspan=2, pady=(6, 0), sticky='e')
        self.takeoff_altitude = tk.StringVar(value='10')
        self.altitude_entry = ttk.Entry(mode_box, textvariable=self.takeoff_altitude, width=10)
        self.altitude_entry.grid(row=1, column=3, padx=5, pady=(6, 0), sticky='ew')
        self.takeoff_button = ttk.Button(mode_box, text='Takeoff', command=self.takeoff)
        self.takeoff_button.grid(row=1, column=4, padx=3, pady=(6, 0), sticky='ew')

        rc_box = ttk.LabelFrame(frame, text='RC channels — received signals', padding=6)
        rc_box.grid(row=3, column=0, columnspan=3, sticky='ew', pady=(0, 6))
        rc_box.columnconfigure(1, weight=1)
        self.channel_bars = {}
        self.channel_text = {}
        self.override_text = {}
        for row, (channel, label, keys) in enumerate(self.CHANNELS):
            ttk.Label(rc_box, text='CH{}  {}'.format(channel, label), width=14).grid(row=row, column=0, sticky='w', pady=3)
            bar = ttk.Progressbar(rc_box, orient='horizontal', maximum=1000, length=220)
            bar.grid(row=row, column=1, sticky='ew', padx=8, pady=3)
            variable = tk.StringVar(value='—')
            target = tk.StringVar(value='Released')
            ttk.Label(rc_box, textvariable=variable, width=10).grid(row=row, column=2, sticky='e')
            ttk.Label(rc_box, textvariable=target, width=18).grid(row=row, column=3, sticky='w', padx=(10, 0))
            self.channel_bars[channel] = bar
            self.channel_text[channel] = variable
            self.override_text[channel] = target

        keyboard_box = ttk.LabelFrame(frame, text='Keyboard RC', padding=6)
        keyboard_box.grid(row=4, column=0, columnspan=3, sticky='ew')
        keyboard_box.columnconfigure(0, weight=1)
        self.keyboard_button = ttk.Button(keyboard_box, text='Enable keyboard RC', command=self.toggle_keyboard)
        self.keyboard_button.grid(row=0, column=0, sticky='w')
        ttk.Button(keyboard_box, text='Pause / hold (Esc)', command=lambda: self.release_keyboard('Keyboard RC paused.')).grid(
            row=0, column=1, sticky='e')
        self.keyboard_pad = tk.Frame(keyboard_box, takefocus=True, borderwidth=2, relief='groove', background='#eef2f5')
        self.keyboard_pad.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(5, 0))
        tk.Label(self.keyboard_pad, text='W / S: throttle up / down     A / D: yaw left / right\n'
                 '← / →: roll left / right     ↑ / ↓: pitch forward / back',
                 background='#eef2f5', padx=10, pady=6).pack()
        self.keyboard_pad.bind('<Button-1>', lambda event: self.keyboard_pad.focus_set())
        for child in self.keyboard_pad.winfo_children():
            child.bind('<Button-1>', lambda event: self.keyboard_pad.focus_set())
        ttk.Label(keyboard_box, text='W/S keeps throttle; other axes center on release. Esc pauses; Loiter holds height.').grid(
            row=2, column=0, columnspan=2, sticky='w', pady=(5, 0))
        self.status = tk.StringVar(value='Disconnected')
        ttk.Label(root, textvariable=self.status, wraplength=860, padding=(10, 6)).grid(row=1, column=0, sticky='ew')
        root.bind('<KeyPress>', self._key_pressed, add='+')
        root.bind('<KeyRelease>', self._key_released, add='+')
        root.bind('<FocusOut>', lambda event: root.after_idle(self._check_keyboard_focus), add='+')
        root.bind('<Destroy>', self._on_destroy, add='+')
        root.protocol('WM_DELETE_WINDOW', self.exit)
        root.update_idletasks()
        required_width = root.winfo_reqwidth()
        required_height = root.winfo_reqheight()
        root.minsize(max(840, required_width), max(560, required_height))
        root.geometry('{}x{}'.format(max(900, required_width), max(600, required_height)))
        self.update_buttons()
        self._poll_job = root.after(100, self.poll)

    def update_buttons(self, snapshot=None):
        if self._destroyed:
            return
        snapshot = snapshot or self.controller.snapshot()
        healthy = snapshot['connected'] and not self.closing
        idle = healthy and not self.busy
        for button in self.mode_buttons:
            button.configure(state='normal' if healthy and (not self.busy or self._operation == 'takeoff') else 'disabled')
        for button in (self.arm_button, self.takeoff_button):
            button.configure(state='normal' if idle else 'disabled')
        self.altitude_entry.configure(state='normal' if not self.busy else 'disabled')
        self.keyboard_button.configure(text='Pause keyboard RC' if self.rc.enabled else 'Enable keyboard RC',
                                       state='normal' if idle else 'disabled')
        self.connect_button.configure(state='normal' if not snapshot['attached'] and not self.busy and not self.closing else 'disabled')
        self.entry.configure(state='normal' if not snapshot['attached'] and not self.busy else 'disabled')
        self.close_button.configure(state='normal' if snapshot['attached'] and not self.closing else 'disabled')

    def _start(self, kind, message, operation):
        if self.busy or self._destroyed:
            return
        self.busy = True
        self._operation = kind
        self._cancel = threading.Event()
        self._job_id += 1
        job = self._job_id
        token = self._cancel
        self.status.set(message)
        self.update_buttons()

        def worker():
            try:
                result = operation(token)
                self.events.put(('done', job, True, result))
            except Exception as exc:
                self.events.put(('done', job, False, exc))

        threading.Thread(target=worker, daemon=True).start()

    def connect(self):
        address = self.connection.get().strip()
        if not address:
            self.status.set('Enter a connection string first.')
            return

        def operation(cancel):
            from dronekit import Vehicle, connect
            opened = []

            class TrackedVehicle(Vehicle):
                def __init__(self, handler):
                    super().__init__(handler)
                    opened.append(self)

            try:
                vehicle = connect(address, wait_ready=False, heartbeat_timeout=15, vehicle_class=TrackedVehicle)
                vehicle.wait_ready('mode', 'armed', timeout=30)
                if cancel.is_set():
                    raise OperationCancelled('Connection cancelled.')
                age = vehicle.last_heartbeat
                if age is None or age >= 5:
                    raise RuntimeError('No recent heartbeat. Check SITL and the MAVProxy output port.')
                return vehicle
            except Exception:
                for vehicle in opened:
                    vehicle.close()
                raise

        self._start('connect', 'Connecting…', operation)

    def _interrupt(self, pending):
        self.release_keyboard(force_release=pending[0] == 'disconnect')
        if self.busy:
            self._pending = pending
            self._cancel.set()
            self.status.set('Cancelling current operation…')
            return True
        return False

    def change_mode(self, name):
        # Validate BEFORE touching current RC: a refused mode must not drop thrust.
        try:
            self.controller.validate_mode_change(name)
        except RuntimeError as exc:
            self.status.set(str(exc))
            return
        if self._interrupt(('mode', name)):
            return
        snapshot = self.controller.snapshot()
        if snapshot['armed'] and name.upper() in ('LOITER', 'ALT_HOLD', 'POSHOLD'):
            # Establish neutral pilot input before leaving GUIDED. Otherwise the
            # simulator's default CH3=1000 immediately commands a descent.
            try:
                throttle = 1500
                if snapshot['mode'] == 'STABILIZE':
                    # Retain manual thrust until Loiter is actually observed.
                    throttle = (self._hold_values or snapshot['channels']).get('3')
                    if throttle is None:
                        raise RuntimeError('Current manual throttle is unknown; wait for RC telemetry.')
                self._hold_values = {str(i): 1500 for i in range(1, 5)}
                self._hold_values['3'] = throttle
                self._hold_mode = snapshot['mode']
                self._send_overrides(self._hold_values, 'Hold')
            except Exception as exc:
                self._hold_values = None
                self._hold_mode = None
                self.status.set('Could not prepare RC handoff: ' + str(exc))
                return
        self._start('mode', 'Requesting ' + name + '…', lambda cancel: self.controller.change_mode(name, cancel))

    def arm(self):
        self.release_keyboard()
        self._start('arm', 'Arming…', lambda cancel: self.controller.arm(cancel))

    def takeoff(self):
        try:
            altitude = validate_altitude(self.takeoff_altitude.get())
        except ValueError as exc:
            self.status.set(str(exc))
            return
        self.release_keyboard()
        job = self._job_id + 1

        def operation(cancel):
            return self.controller.takeoff(altitude, cancel,
                progress=lambda text: self.events.put(('progress', job, text)))

        self._start('takeoff', 'Preparing takeoff…', operation)

    def disconnect(self):
        if self._interrupt(('disconnect', None)):
            return
        self._start('disconnect', 'Closing connection…', lambda cancel: self.controller.disconnect())

    def toggle_keyboard(self):
        if self.rc.enabled:
            self.release_keyboard('Keyboard RC paused.')
            return
        snapshot = self.controller.snapshot()
        if not snapshot['connected'] or self.busy:
            self.status.set('Connect to a healthy vehicle before enabling keyboard RC.')
            return
        if snapshot['mode'] not in self.controller.MANUAL_MODES:
            self.status.set('Choose Loiter or Stabilize for keyboard RC. Guided is used for takeoff.')
            return
        if not snapshot['armed']:
            self.status.set('Arm the vehicle before enabling keyboard RC.')
            return
        # In altitude-hold modes 1500 is neutral, even if the last RC throttle was low.
        throttle = (1500 if snapshot['mode'] != 'STABILIZE' else
                    (self._hold_values or snapshot['channels']).get('3'))
        try:
            self.rc.enable(throttle)
        except ValueError as exc:
            self.status.set(str(exc))
            return
        self._keyboard_mode = snapshot['mode']
        self._last_tick = time.monotonic()
        self.keyboard_pad.focus_set()
        self.keyboard_pad.configure(background='#dcefdc')
        self._hold_values = None
        self._hold_mode = None
        self.status.set('Keyboard RC enabled. W/S adjusts throttle; Esc pauses and centers the sticks.')
        self.update_buttons()

    def _send_overrides(self, values, label):
        self._rc_owned = True
        self.controller.vehicle.channels.overrides = dict(values)
        if not self._destroyed:
            for channel, value in values.items():
                self.override_text[channel].set('{}: {} µs'.format(label, value))

    def release_keyboard(self, reason=None, force_release=False):
        previous = self.rc.values or self._hold_values or {}
        previous_mode = self._hold_mode
        owned = self._rc_owned or self.rc.enabled or self._hold_values is not None
        self.rc.disable()
        self._keyboard_mode = None
        for job in self._keyups.values():
            self.root.after_cancel(job)
        self._keyups.clear()
        self._hold_values = None
        self._hold_mode = None
        snapshot = self.controller.snapshot()
        if (owned and not force_release and not self.closing and not self._destroyed and
                snapshot['connected'] and snapshot['armed'] and
                snapshot['mode'] in self.controller.MANUAL_MODES):
            # Releasing overrides to the simulator's low throttle is NOT a hover.
            # Altitude-hold modes need mid-stick; manual Stabilize retains the
            # operator's established throttle, without claiming to hold height.
            throttle = 1500 if snapshot['mode'] != 'STABILIZE' else previous.get('3')
            if throttle is not None:
                self._hold_values = {str(i): 1500 for i in range(1, 5)}
                self._hold_values['3'] = throttle
                self._hold_mode = snapshot['mode']
        elif (owned and not force_release and not self.closing and not self._destroyed and
                snapshot['connected'] and snapshot['armed'] and self.busy and
                self._operation == 'mode' and snapshot['mode'] == previous_mode):
            # Pause must not undo a prepared automatic-to-Loiter bridge while
            # the mode command is still awaiting confirmation.
            self._hold_values = dict(previous)
            self._hold_mode = previous_mode
        failed = False
        try:
            if self._hold_values is not None:
                self._send_overrides(self._hold_values, 'Hold')
            elif owned and self.controller.vehicle is not None:
                self.controller.vehicle.channels.overrides = {}
                self._rc_owned = False
        except Exception as exc:
            failed = True
            self._hold_values = None
            self._hold_mode = None
            if not self._destroyed:
                self.status.set('Could not update RC pause/release: ' + str(exc))
        if not self._destroyed:
            self.keyboard_pad.configure(background='#fff1ce' if self._hold_values else '#eef2f5')
            if self._hold_values is None:
                for variable in self.override_text.values():
                    variable.set('Released')
            if reason and not failed:
                if self._hold_mode in ('LOITER', 'ALT_HOLD', 'POSHOLD'):
                    reason += ' Neutral RC is holding altitude.'
                elif self._hold_mode == 'STABILIZE':
                    reason += ' Manual throttle retained; altitude is not held.'
                self.status.set(reason)
            self.update_buttons()

    def _key_pressed(self, event):
        key = event.keysym.lower()
        if key == 'escape':
            self.release_keyboard('Keyboard RC paused.')
            return 'break'
        if not self.rc.enabled or self.root.focus_get() is not self.keyboard_pad or event.state & 0x0c:
            return None
        if key not in self.rc.KEYS:
            return None
        pending = self._keyups.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        self.rc.press(key)
        return 'break'

    def _key_released(self, event):
        key = event.keysym.lower()
        if not self.rc.enabled or key not in self.rc.KEYS:
            return None
        pending = self._keyups.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)

        def release():
            self._keyups.pop(key, None)
            self.rc.release(key)

        # X11 can generate Release/Press pairs for auto-repeat. A matching press
        # cancels this short delayed release, so held keys remain steady.
        self._keyups[key] = self.root.after(35, release)
        return 'break'

    def _check_keyboard_focus(self):
        if not self._destroyed and self.rc.enabled and self.root.focus_get() is not self.keyboard_pad:
            self.release_keyboard('Keyboard RC released: control pad lost focus.')

    def _tick_rc(self, snapshot):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if self.rc.enabled:
            if (not snapshot['connected'] or not snapshot['armed'] or self.busy or
                    snapshot['mode'] != self._keyboard_mode or self.root.focus_get() is not self.keyboard_pad):
                self.release_keyboard('Keyboard RC paused: focus, vehicle state or connection changed.',
                                      force_release=not snapshot['connected'] or not snapshot['armed'])
            else:
                try:
                    self._send_overrides(self.rc.update(dt), 'Target')
                except Exception as exc:
                    self.release_keyboard('Keyboard RC failed: ' + str(exc), force_release=True)
                return
        if self._hold_values is None:
            return
        if snapshot['mode'] in ('LOITER', 'ALT_HOLD', 'POSHOLD') and snapshot['mode'] != self._hold_mode:
            # Center throttle only once the altitude-hold mode has taken over.
            self._hold_values = {str(i): 1500 for i in range(1, 5)}
            self._hold_mode = snapshot['mode']
        preparing_mode = self.busy and self._operation == 'mode'
        if (not snapshot['connected'] or not snapshot['armed'] or self.closing or
                snapshot['mode'] != self._hold_mode or
                (self._hold_mode not in self.controller.MANUAL_MODES and not preparing_mode)):
            self.release_keyboard(force_release=True)
            return
        try:
            # Keep neutral/held input alive even with keyboard off or out of focus.
            # Automatic modes get overrides released only AFTER the mode changes.
            self._send_overrides(self._hold_values, 'Hold')
        except Exception as exc:
            self.release_keyboard('RC hold failed: ' + str(exc), force_release=True)

    def _drain_events(self):
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                return
            if event[1] != self._job_id:
                continue
            if event[0] == 'progress':
                if not self.closing and self._pending is None:
                    self.status.set(event[2])
                continue
            _, _, success, result = event
            kind = self._operation
            self.busy = False
            self._operation = None
            self._cancel = None
            if success:
                if kind == 'connect':
                    try:
                        self.controller.attach(result)
                        self.status.set('Connected')
                    except Exception as exc:
                        result.close()
                        self.status.set('Error: ' + str(exc))
                elif kind == 'takeoff':
                    self.status.set('Takeoff reached {:.1f} m.'.format(result))
                elif kind == 'disconnect':
                    self.status.set('Disconnected')
                elif kind == 'arm':
                    self.status.set('Vehicle armed')
                else:
                    if self.controller.snapshot()['mode'] in ('LOITER', 'ALT_HOLD', 'POSHOLD') and self._hold_values is not None:
                        self.status.set('Mode change confirmed. Neutral RC holds altitude; enable keyboard RC to fly.')
                    else:
                        self.status.set('Mode change confirmed')
            else:
                self.status.set(str(result) if isinstance(result, OperationCancelled) else 'Error: ' + str(result))

    def poll(self):
        if self._destroyed:
            return
        if self._poll_job is not None:
            self.root.after_cancel(self._poll_job)
            self._poll_job = None
        self._drain_events()
        notices = self.controller.drain_notices()
        if notices and not self.closing and self._pending is None:
            self.status.set('Vehicle: ' + ' | '.join(notices[-3:]))
        if not self.busy:
            if self.closing:
                if self.controller.vehicle is not None:
                    self.disconnect()
                else:
                    self.root.destroy()
                    return
            elif self._pending is not None:
                kind, value = self._pending
                self._pending = None
                self.disconnect() if kind == 'disconnect' else self.change_mode(value)
        snapshot = self.controller.snapshot()
        connected = snapshot['connected']
        self.connection_status.set('Connected' if connected else ('Connection lost' if snapshot['attached'] else 'Disconnected'))
        age = snapshot['heartbeat_age']
        self.heartbeat_text.set('—' if age is None else '{:.1f} s ago'.format(age))
        self.mode_text.set((snapshot['mode'] or '—') if connected else '—')
        self.armed_text.set(('Armed' if snapshot['armed'] else 'Disarmed') if connected and snapshot['armed'] is not None else '—')
        altitude = snapshot['altitude']
        self.altitude_text.set('{:.1f} m'.format(altitude) if connected and altitude is not None else '—')
        for channel, value in snapshot['channels'].items():
            value = value if connected else None
            self.channel_text[channel].set('—' if value is None else '{} µs'.format(value))
            self.channel_bars[channel]['value'] = 0 if value is None else max(0, min(1000, value - 1000))
        self._tick_rc(snapshot)
        self.update_buttons(snapshot)
        self._poll_job = self.root.after(100, self.poll)

    def exit(self):
        self.closing = True
        self._pending = None
        self.release_keyboard(force_release=True)
        if self._cancel is not None:
            self._cancel.set()
        if not self.busy and self.controller.vehicle is None:
            self.root.destroy()

    def _on_destroy(self, event):
        if event.widget is not self.root:
            return
        self._destroyed = True
        if self._cancel is not None:
            self._cancel.set()
        if self._poll_job is not None:
            self.root.after_cancel(self._poll_job)
            self._poll_job = None
        self.release_keyboard(force_release=True)


if __name__ == '__main__':
    window = tk.Tk()
    ControlPanel(window)
    window.mainloop()
