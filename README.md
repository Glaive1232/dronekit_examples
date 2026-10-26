# dronekit-gazebo-sitl

Ubuntu + ArduPilot SITL + Gazebo + MAVProxy + dronekit, for running the example scripts against a simulated drone.

## Build

```
docker build -t dronekit-gazebo-sitl .
```

## Run

### Windows

Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) and launch it with XLaunch: Multiple windows → Start no client → check "Disable access control". Leave it running.

```
docker run -it --rm -v "<path to examples folder>:/home/ardupilot/dronekit_examples" -e DISPLAY=host.docker.internal:0.0 --name dk dronekit-gazebo-sitl
```

### Linux

```
xhost +local:docker
docker run -it --rm -v "<path to examples folder>:/home/ardupilot/dronekit_examples" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name dk dronekit-gazebo-sitl
```

- `-v ...` mounts your `examples/` folder into the container, so edits/new files show up live, no rebuild needed.
- `-e DISPLAY=...` gives you the Gazebo GUI and MAVProxy map. Drop it (and the X server setup) to run headless.
- `--rm` deletes the container when you exit, so names never clash next time.

Inside the container, start the simulator:

```
~/start_sitl.sh
```

For a lightweight run with no Gazebo and no windows (SITL only, built-in physics), add the flag:

```
~/start_sitl.sh --no-gui
```

Wait ~30 seconds for it to boot (EKF init, GPS lock, etc).

## Running examples

Open a second terminal:

```
docker exec -it dk bash
python3 01_connect_sitl.py
```

## Control panel (GUI)

`examples/control_panel.py` is a small Tkinter app: connection string field, Connect / Close Connection, mode buttons (Guided, Stabilize, Loiter, Land, RTL) and Arm. It needs the X display, so start the container with the `DISPLAY` options above, then:

```
docker exec -it dk python3 control_panel.py
```

## Rebuilding

Only needed if you change the `Dockerfile` itself (new package, etc). Editing files in `examples/` never needs a rebuild.

## Run only SITL (no Gazebo)

```
sim_vehicle.py -v ArduCopter  --out udp:127.0.0.1:14550 --map --console --no-rebuild
```
