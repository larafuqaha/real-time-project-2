# Real-Time Traffic Light Control System Using IPC
**Birzeit University — ENCS4330 (Real-Time Applications & Embedded Systems)**
**Project #2 — 2nd semester 2025/2026**

## 1. What this project does
This is a multi-process simulation of a real-world four-way intersection.
Each of the major sub-systems (intersection controller, the four traffic
lights, vehicle detectors, pedestrian buttons, emergency-vehicle detector,
event logger, and a 3-D OpenGL visualiser) is its own Linux process.
They cooperate using three classic System V IPC mechanisms:

| Resource         | IPC mechanism      | Why                                                       |
|------------------|--------------------|-----------------------------------------------------------|
| Global state     | **Shared memory**  | Every process reads it; copying it in messages is wasteful|
| Events / commands| **Message queues** | Asynchronous, FIFO, never lost, with mtype filtering      |
| State protection | **Semaphore**      | Mutex over the shared blackboard (prevents races)         |

## 2. Repository layout
```
traffic_system/
├── Makefile               Build rules for every process
├── README.md              This file
├── config/
│   └── system.cfg         All tunable parameters (no hard-coding)
├── docs/
│   ├── design_report.pdf  Why-we-chose-what design document
│   └── study_guide.docx   Walk-through of every file in detail
├── logs/
│   └── system.log         Created at runtime by the logger
└── src/
    ├── common.h           Shared types, message structs, helpers
    ├── ipc_init.h         Wrappers around shmget/semget/msgget
    ├── config.h           Plain-text config-file parser
    ├── main.c             Launcher — forks every component
    ├── controller.c       Phase state machine, safety, adaptive timing
    ├── traffic_light.c    One per direction (N, S, E, W)
    ├── vehicle_detector.c Random vehicle arrival simulator
    ├── pedestrian.c       Random pedestrian-button presses
    ├── emergency.c        Rare emergency vehicle generator
    ├── logger.c           Drains the log queue to file + screen
    └── gui_opengl.c       freeglut 3-D visualisation of intersection
```

## 3. Building

### 3.1 Prerequisites (Ubuntu / Debian)
```bash
sudo apt update
sudo apt install build-essential freeglut3-dev libglu1-mesa-dev
```

### 3.2 Build everything
```bash
make            # release build (-O2)
make debug      # build with -g for gdb
```
If you don't have OpenGL/freeglut installed you can still build all the
back-end processes:
```bash
make main logger controller traffic_light vehicle_detector pedestrian emergency
```

## 4. Running

### 4.1 Normal run (with GUI)
```bash
./main config/system.cfg
```
A window opens showing the intersection. The terminal prints status
information; detailed events are appended to `logs/system.log`.

### 4.2 Headless (no OpenGL window)
```bash
./main config/system.cfg --no-gui
```

### 4.3 Stopping
- Press **q** or **Esc** in the OpenGL window, or
- Press **Ctrl-C** in the terminal.

`main` will cleanly stop every child process and remove the IPC resources
it created (shared memory, semaphore, all three message queues).

### 4.4 Interactive controls (GUI window)
| Key                  | Action                                    |
|----------------------|-------------------------------------------|
| `n` `s` `e` `w`      | Inject a vehicle from N/S/E/W lane        |
| `N` `S` `E` `W`      | Press pedestrian button at that side      |
| `1` `2` `3` `4`      | Trigger emergency vehicle from N/S/E/W    |
| arrow keys           | Rotate the camera                         |
| `+` / `-`            | Zoom in / out                             |
| `h`                  | Toggle the on-screen help                 |
| `q` / `Esc`          | Quit                                      |

## 5. Configuration file
All timing parameters and probabilities live in `config/system.cfg`.
Change values there and re-run **without** recompiling. The keys are
documented inside the file itself.

## 6. Troubleshooting
- **"IPC attach failed"** — a previous run did not clean up. Run
  `make ipcclean` to remove every dangling SysV IPC owned by you, or
  `ipcs` to inspect them manually.
- **Black screen in OpenGL** — make sure `freeglut3-dev` and a working
  GL driver are installed. Try `glxgears` to verify your GL stack.
- **Cars look stuck** — increase `VEHICLE_SPAWN_RATE` or extend
  `T_GREEN_MAX`. The controller will adaptively give more green time
  to busier lanes.

## 7. Real-time properties verified at runtime
- Two conflicting greens are *never* both green (checked every cycle).
- Yellow always precedes red on a transitioning direction.
- Pedestrian crossing only opens while every vehicle light is red.
- Emergencies safely yellow-out the previous direction before granting
  priority to the emergency lane.
- All processes share state through a single semaphore-protected
  blackboard — no race conditions possible on the data structure.
- The logger uses `IPC_NOWAIT` on the producer side so the real-time
  loops never block waiting for disk I/O.
