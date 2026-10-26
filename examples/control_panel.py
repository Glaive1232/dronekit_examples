"""Tkinter control panel for the DroneKit SITL examples (Python 3.9)."""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk


class ControlPanel:
    MODES = ("Guided", "Stabilize", "Loiter", "Land", "RTL")

    def __init__(self, root):
        self.root = root
        self.vehicle = None
        self.busy = False
        self.closing = False
        self.events = queue.Queue()
        root.title("DroneKit Control Panel")
        root.resizable(False, False)
        frame = ttk.Frame(root, padding=20)
        frame.grid()
        ttk.Label(frame, text="Connection string").grid(row=0, column=0, sticky="w")
        self.connection = tk.StringVar(value="udpin:0.0.0.0:14450")
        self.entry = ttk.Entry(frame, textvariable=self.connection, width=36)
        self.entry.grid(row=1, column=0, columnspan=2, pady=(4, 12), sticky="ew")
        self.connect_button = ttk.Button(frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=1, column=2, padx=(8, 0))
        self.controls = []
        for index, mode in enumerate(self.MODES):
            button = ttk.Button(frame, text=mode, command=lambda name=mode: self.change_mode(name))
            button.grid(row=2 + index // 3, column=index % 3, padx=4, pady=5, sticky="ew")
            self.controls.append(button)
        self.arm_button = ttk.Button(frame, text="Arm", command=self.arm)
        self.arm_button.grid(row=3, column=2, padx=4, pady=5, sticky="ew")
        self.controls.append(self.arm_button)
        self.close_button = ttk.Button(frame, text="Close Connection", command=self.disconnect)
        self.close_button.grid(row=4, column=0, columnspan=3, pady=10, sticky="ew")
        self.status = tk.StringVar(value="Disconnected")
        ttk.Label(frame, textvariable=self.status, wraplength=420).grid(row=5, column=0, columnspan=3, sticky="w")
        self.telemetry = tk.StringVar(value="Mode: —    Armed: —")
        ttk.Label(frame, textvariable=self.telemetry).grid(row=6, column=0, columnspan=3, sticky="w", pady=(8, 0))
        root.protocol("WM_DELETE_WINDOW", self.exit)
        self.update_buttons()
        root.after(100, self.poll)

    def update_buttons(self):
        connected = self.vehicle is not None
        for button in self.controls:
            button.configure(state="normal" if connected and not self.busy else "disabled")
        self.close_button.configure(state="normal" if connected and not self.busy else "disabled")
        self.connect_button.configure(state="normal" if not connected and not self.busy else "disabled")
        self.entry.configure(state="normal" if not connected and not self.busy else "disabled")

    def run(self, message, operation):
        if self.busy:
            return
        self.busy = True
        self.status.set(message)
        self.update_buttons()

        def worker():
            try:
                result = operation()
                self.events.put((True, result))
            except Exception as exc:
                self.events.put((False, str(exc)))

        threading.Thread(target=worker, daemon=True).start()

    def connect(self):
        address = self.connection.get().strip()
        if not address:
            self.status.set("Enter a connection string first.")
            return

        def operation():
            from dronekit import connect
            vehicle = connect(address, wait_ready=False, heartbeat_timeout=15)
            try:
                # Only these attributes are needed by this panel. Waiting for
                # GPS, attitude and every parameter can reject a usable link.
                vehicle.wait_ready("mode", "armed", timeout=30)
                heartbeat_age = vehicle.last_heartbeat
                if heartbeat_age is None or heartbeat_age >= 5:
                    raise RuntimeError(
                        "No recent heartbeat. Check SITL and the MAVProxy output port."
                    )
            except Exception:
                vehicle.close()
                raise
            return ("connected", vehicle)

        self.run("Connecting…", operation)

    @staticmethod
    def wait_until(predicate, description, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.2)
        raise TimeoutError(description + " was not confirmed by the vehicle.")

    def change_mode(self, name):
        if self.vehicle is None:
            return
        vehicle = self.vehicle

        def operation():
            from dronekit import VehicleMode
            vehicle.mode = VehicleMode(name.upper())
            self.wait_until(lambda: vehicle.mode.name == name.upper(), name)
            return ("message", "Mode changed to " + name)

        self.run("Requesting " + name + "…", operation)

    def arm(self):
        if self.vehicle is None:
            return
        vehicle = self.vehicle

        def operation():
            if not vehicle.is_armable:
                raise RuntimeError("Vehicle is not armable. Check pre-arm status in the simulator.")
            vehicle.armed = True
            self.wait_until(lambda: vehicle.armed, "Arming")
            return ("message", "Vehicle armed")

        self.run("Arming…", operation)

    def disconnect(self):
        if self.vehicle is None:
            return
        vehicle = self.vehicle

        def operation():
            vehicle.close()
            return ("disconnected", None)

        self.run("Closing connection…", operation)

    def exit(self):
        self.closing = True
        if not self.busy:
            if self.vehicle is not None:
                self.disconnect()
            else:
                self.root.destroy()

    def poll(self):
        try:
            success, result = self.events.get_nowait()
        except queue.Empty:
            pass
        else:
            self.busy = False
            if success:
                kind, value = result
                if kind == "connected":
                    self.vehicle = value
                    self.status.set("Connected")
                elif kind == "disconnected":
                    self.vehicle = None
                    self.status.set("Disconnected")
                else:
                    self.status.set(value)
            else:
                self.status.set("Error: " + result)
                # Keep the window open so a failed close can be retried.
                self.closing = False
            self.update_buttons()
        if self.closing and not self.busy:
            self.exit()
            if self.vehicle is None:
                return
        if self.vehicle is not None:
            self.telemetry.set("Mode: {}    Armed: {}".format(self.vehicle.mode.name, self.vehicle.armed))
        else:
            self.telemetry.set("Mode: —    Armed: —")
        self.root.after(100, self.poll)


if __name__ == "__main__":
    window = tk.Tk()
    ControlPanel(window)
    window.mainloop()
