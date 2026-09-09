#!/usr/bin/env python3
"""
Autonomous Storage & Sorting Matrix Pipeline
Hardware: uFactory xArm (xArm-Python-SDK)
Author: Darius Vasilache
"""

import time
import sys
from xarm.wrapper import XArmAPI

# Hardware mapping: Coordinates, CI input sensors, and CO output LEDs
SLOT_CONFIG = {
    1: {"ci": 4, "co": 4, "coords": [208, 0]},
    2: {"ci": 2, "co": 2, "coords": [151, 65]},
    3: {"ci": 3, "co": 3, "coords": [208, -65]},
    4: {"ci": 0, "co": 0, "coords": [151, -65]},
    5: {"ci": 5, "co": 5, "coords": [208, 65]},
    6: {"ci": 1, "co": 1, "coords": [151, 0]},
}

# Feeder input ports
INPUT_PORTS = {
    "CI_6": {"ci": 6, "coords": [288, -49]},
    "CI_7": {"ci": 7, "coords": [288, 8]}
}

Z_SAFE = 80.0
Z_WORK = 36.0
TOOL_ORIENTATION = [180.0, 0.0, 0.0]  # Roll, Pitch, Yaw
SPEED = 150
WAIT_ACTION = 3.0


class XArmMatrixController:
    def __init__(self, ip: str = "192.168.1.196"):
        # Connect and initialize robot arm
        self.arm = XArmAPI(ip)
        self.arm.clean_error()
        self.arm.clean_warn()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        time.sleep(1)

    def go_home(self):
        # Move arm to initial position
        self.arm.move_gohome(wait=True)

    def set_gripper(self, open_gripper: bool):
        # Open or close gripper
        if open_gripper:
            if hasattr(self.arm, 'set_vacuum_gripper'):
                self.arm.set_vacuum_gripper(False)
            else:
                self.arm.open_lite_gripper()
        else:
            if hasattr(self.arm, 'set_vacuum_gripper'):
                self.arm.set_vacuum_gripper(True)
            else:
                self.arm.close_lite_gripper()
        time.sleep(WAIT_ACTION)

    def read_ci(self, port: int) -> int:
        # Read digital input: 0 = block detected, 1 = empty
        code, val = self.arm.get_tgpio_digital(port) if port < 2 else self.arm.get_cgpio_digital(port)
        return val

    def set_co(self, port: int, state: bool):
        # Set digital output LED state (HIGH/LOW)
        val = 1 if state else 0
        if port < 2:
            self.arm.set_tgpio_digital(port, val)
        else:
            self.arm.set_cgpio_digital(port, val)

    def update_leds(self):
        # Step 4: Synchronize all LEDs live with current sensor readings
        for data in SLOT_CONFIG.values():
            occupied = (self.read_ci(data["ci"]) == 0)
            self.set_co(data["co"], occupied)

    def pick_and_place(self, source_coords, target_coords, source_co=None, target_co=None):
        # 1. Approach and pick block
        self.arm.set_position(x=source_coords[0], y=source_coords[1], z=Z_SAFE, *TOOL_ORIENTATION, speed=SPEED, wait=True)
        self.arm.set_position(x=source_coords[0], y=source_coords[1], z=Z_WORK, *TOOL_ORIENTATION, speed=SPEED, wait=True)
        self.set_gripper(open_gripper=False)

        # Turn off source LED immediately upon pick
        if source_co is not None:
            self.set_co(source_co, False)

        # 2. Lift to safe height
        self.arm.set_position(x=source_coords[0], y=source_coords[1], z=Z_SAFE, *TOOL_ORIENTATION, speed=SPEED, wait=True)

        # 3. Move and place block at destination
        self.arm.set_position(x=target_coords[0], y=target_coords[1], z=Z_SAFE, *TOOL_ORIENTATION, speed=SPEED, wait=True)
        self.arm.set_position(x=target_coords[0], y=target_coords[1], z=Z_WORK, *TOOL_ORIENTATION, speed=SPEED, wait=True)
        self.set_gripper(open_gripper=True)

        # Turn on target LED immediately upon place
        if target_co is not None:
            self.set_co(target_co, True)

        # 4. Return to safe height and go home
        self.arm.set_position(x=target_coords[0], y=target_coords[1], z=Z_SAFE, *TOOL_ORIENTATION, speed=SPEED, wait=True)
        self.go_home()

    def get_first_free_slot(self, ascending: bool = True):
        # Find first empty slot (ascending 1->6 or descending 6->1)
        order = range(1, 7) if ascending else range(6, 0, -1)
        for slot in order:
            if self.read_ci(SLOT_CONFIG[slot]["ci"]) == 1:
                return slot
        return None

    def run(self):
        # Initial boot sequence: open gripper, home, and turn on LEDs for occupied slots
        self.set_gripper(open_gripper=True)
        self.go_home()
        self.update_leds()

        while True:
            # Step 1: Internal sorting / compacting (6 down to 1)
            for curr_slot in range(6, 0, -1):
                if self.read_ci(SLOT_CONFIG[curr_slot]["ci"]) == 0:
                    free_slot = self.get_first_free_slot(ascending=True)
                    if free_slot is not None and free_slot < curr_slot:
                        self.pick_and_place(
                            source_coords=SLOT_CONFIG[curr_slot]["coords"],
                            target_coords=SLOT_CONFIG[free_slot]["coords"],
                            source_co=SLOT_CONFIG[curr_slot]["co"],
                            target_co=SLOT_CONFIG[free_slot]["co"]
                        )

            # Step 2: Feed from CI 6 (Priority: 1 -> 6)
            if self.read_ci(INPUT_PORTS["CI_6"]["ci"]) == 0:
                target_slot = self.get_first_free_slot(ascending=True)
                if target_slot is not None:
                    self.pick_and_place(
                        source_coords=INPUT_PORTS["CI_6"]["coords"],
                        target_coords=SLOT_CONFIG[target_slot]["coords"],
                        target_co=SLOT_CONFIG[target_slot]["co"]
                    )
                else:
                    # Matrix full: Move forward 100mm, wait, and return home
                    pos = self.arm.position
                    self.arm.set_position(x=pos[0] + 100, y=pos[1], z=Z_SAFE, *TOOL_ORIENTATION, wait=True)
                    time.sleep(2)
                    self.go_home()

            # Step 3: Feed from CI 7 (Priority: 6 -> 1)
            elif self.read_ci(INPUT_PORTS["CI_7"]["ci"]) == 0:
                target_slot = self.get_first_free_slot(ascending=False)
                if target_slot is not None:
                    self.pick_and_place(
                        source_coords=INPUT_PORTS["CI_7"]["coords"],
                        target_coords=SLOT_CONFIG[target_slot]["coords"],
                        target_co=SLOT_CONFIG[target_slot]["co"]
                    )
                else:
                    # Matrix full: Stop above CI 7 feeder
                    self.arm.set_position(
                        x=INPUT_PORTS["CI_7"]["coords"][0],
                        y=INPUT_PORTS["CI_7"]["coords"][1],
                        z=Z_SAFE,
                        *TOOL_ORIENTATION,
                        wait=True
                    )

            # Keep LEDs updated
            self.update_leds()
            time.sleep(0.5)


if __name__ == "__main__":
    robot = XArmMatrixController()
    try:
        robot.run()
    except KeyboardInterrupt:
        robot.go_home()
        sys.exit(0)
