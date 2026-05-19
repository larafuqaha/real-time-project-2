#######################################################################
#  Makefile  —  Real-Time Traffic Light Control System Using IPC
#  ENCS4330 Project #2
#######################################################################
#  Compiles every component into the project root so that main can
#  fork+exec them with the canonical name ./<binary>.
#
#  Build:        make            (release)
#                make debug      (with -g for gdb)
#  Clean:        make clean
#  Run:          make run        (== ./main config/system.cfg)
#######################################################################

CC      := gcc
CFLAGS  := -Wall -Wextra -O2 -std=gnu99 -Isrc
LDFLAGS :=
GLLIBS  := -lglut -lGLU -lGL -lm

# All command-line binaries (no OpenGL)
BINS    := main logger controller traffic_light vehicle_detector \
           pedestrian emergency
GUI_BIN := gui_opengl

# Default target: everything that compiles cleanly on this machine
all: $(BINS) $(GUI_BIN)

# --- core processes ---------------------------------------------------
main:              src/main.c src/common.h src/ipc_init.h src/config.h
	$(CC) $(CFLAGS) -o $@ src/main.c $(LDFLAGS)

logger:            src/logger.c src/common.h src/ipc_init.h
	$(CC) $(CFLAGS) -o $@ src/logger.c $(LDFLAGS)

controller:        src/controller.c src/common.h src/ipc_init.h
	$(CC) $(CFLAGS) -o $@ src/controller.c $(LDFLAGS)

traffic_light:     src/traffic_light.c src/common.h src/ipc_init.h
	$(CC) $(CFLAGS) -o $@ src/traffic_light.c $(LDFLAGS)

vehicle_detector:  src/vehicle_detector.c src/common.h src/ipc_init.h
	$(CC) $(CFLAGS) -o $@ src/vehicle_detector.c $(LDFLAGS)

pedestrian:        src/pedestrian.c src/common.h src/ipc_init.h
	$(CC) $(CFLAGS) -o $@ src/pedestrian.c $(LDFLAGS)

emergency:         src/emergency.c src/common.h src/ipc_init.h
	$(CC) $(CFLAGS) -o $@ src/emergency.c $(LDFLAGS)

# --- OpenGL GUI -------------------------------------------------------
$(GUI_BIN):        src/gui_opengl.c src/common.h src/ipc_init.h
	$(CC) $(CFLAGS) -o $@ src/gui_opengl.c $(GLLIBS) $(LDFLAGS)

# --- debug build (just rebuilds with -g) ------------------------------
debug: CFLAGS := -Wall -Wextra -g -O0 -std=gnu99 -Isrc
debug: clean all

run: all
	./main config/system.cfg

# Run without the GUI (useful if libglut is not installed)
run-nogui: $(BINS)
	./main config/system.cfg --no-gui

clean:
	rm -f $(BINS) $(GUI_BIN)
	rm -f logs/*.log

# Remove orphan IPC resources by user (handy after a crash)
ipcclean:
	-@ipcs -q | awk '/^0x/{print $$2}' | xargs -r -n1 ipcrm -q 2>/dev/null
	-@ipcs -m | awk '/^0x/{print $$2}' | xargs -r -n1 ipcrm -m 2>/dev/null
	-@ipcs -s | awk '/^0x/{print $$2}' | xargs -r -n1 ipcrm -s 2>/dev/null
	@echo "IPC resources cleaned."

.PHONY: all clean debug run run-nogui ipcclean
