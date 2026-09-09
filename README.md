# ufactory-xarm-storage-matrix
Autonomous storage matrix &amp; pick-and-place sorting pipeline for the uFactory xArm using xArm-Python-SDK and digital I/O sensors.
# Autonomous Matrix Sorting & Storage Pipeline (uFactory xArm)

Industrial pick-and-place storage and sorting system implemented for the uFactory xArm collaborative robot using digital I/O sensors and status indicators.

## System Architecture & I/O Mapping
The cell consists of a 2x3 storage matrix (slots 1 to 6) and two feeder trays (CI 6 & CI 7). Sensors use active-low logic (`0` = container detected, `1` = slot available). Indicator LEDs (CO 0 to CO 5) reflect slot occupancy in real time.

| Matrix Slot | Digital Input (Sensor) | Digital Output (LED) | Work Coordinates (X, Y) |
| :---: | :---: | :---: | :---: |
| **Slot 1** | CI 4 | CO 4 | (208, 0) |
| **Slot 2** | CI 2 | CO 2 | (151, 65) |
| **Slot 3** | CI 3 | CO 3 | (208, -65) |
| **Slot 4** | CI 0 | CO 0 | (151, -65) |
| **Slot 5** | CI 5 | CO 5 | (208, 65) |
| **Slot 6** | CI 1 | CO 1 | (151, 0) |
| **Feeder Left** | CI 6 | - | (288, -49) |
| **Feeder Right** | CI 7 | - | (288, 8) |

* **Safe Plane (Z):** 80 mm (collision avoidance)
* **Work Plane (Z):** 36 mm (grip/release height)

## Control Logic & Workflow
1. **Internal Sorting & Compaction:** Scans matrix slots from 6 down to 1. If a container is found, it is dynamically relocated to the lowest available index to ensure continuous high-density packing.
2. **Ascending Feeder Routine (CI 6):** Detects parts on the left feeder and transfers them to the lowest available slot (1 to 6). If the matrix is fully occupied, the arm executes an alert motion (+100 mm on X) and returns to Home.
3. **Descending Feeder Routine (CI 7):** Operates when CI 6 is idle, transferring containers to the highest available index (6 down to 1). If all slots are full, the arm halts directly above CI 7.
4. **Live I/O State Synchronization:** Actuates corresponding LED indicators (CO 0 to CO 5) synchronously upon pick and place actions.

## Stack
* **Robot:** uFactory xArm Manipulator
* **SDK / Environment:** Python 3, `xArm-Python-SDK`
* **Perception / Control:** Industrial limit sensors (CI0-CI7) & digital outputs (CO0-CO5)
