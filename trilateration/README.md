# UWB Trilateration

2D indoor positioning using 5x BU-04 UWB modules and an Arduino Mega.

## Hardware

- 4x BU-04 modules acting as anchors, each with an Arduino Nano ESP32 and battery
- 1x BU-04 module acting as a tag connected to the Arduino Mega
- 1x Arduino Mega running the trilateration code

## Wiring

**Anchors (Arduino Nano ESP32):**
<img width="3508" height="2481" alt="trilaterering_final_oppsett" src="https://github.com/user-attachments/assets/be7cbaa7-8550-4211-ae7d-4e304a0e0b85" />


**Tag (Arduino Mega):**
<img width="4961" height="3508" alt="BU04_til_pc" src="https://github.com/user-attachments/assets/a76333a0-a9f8-49fb-9537-dfa68b2090ec" />


## Anchor and Tag Configuration

Flash each anchor and tag [using the BU-04_flashing code](BU04_FLASHING.ino):

Anchor:
```cpp
configureBU04(0, 1, 1, 1); // anchor 0
configureBU04(1, 1, 1, 1); // anchor 1
configureBU04(2, 1, 1, 1); // anchor 2
configureBU04(3, 1, 1, 1); // anchor 3
```

Tag:

```cpp
configureBU04(0, 0, 1, 1);  // tag 0
```

## Physical Placement

Place anchors at the four corners of the room/container:

```
BS1 (0, y) -------- BS3 (x, y)
|                            |
|                            |
BS0 (0, 0) -------- BS2 (x, 0)
```

Measure x and y in metres from the BS0 corner and update `BS[4][2]` in [](BU04_ROVER_READ.ino).

## Calibration

1. Flash [](BU04_CALIBRATION.ino) to the Mega
2. Place tag at each distance (100mm to 1500mm in 100mm steps)
3. Send any character in Serial Monitor to advance to next step
4. Paste results into Google Sheets, plot measured vs actual
5. Get slope and intercept from trendline
6. Update `CALIB_SLOPE` and `CALIB_INTERCEPT` in `trilateration/trilateration.ino`
7. Repeat until trendline is ~1.0*x + 0

## Visualisation

1. Flash `trilateration/trilateration.ino` to Mega
2. Close Arduino Serial Monitor
3. Open `visualisation/visualisation.pde` in Processing IDE
4. Set correct COM port and base station positions
5. Run — circles show distances from each anchor, dot shows tag position

## Output Format

Serial output at 115200 baud:

```
x,y,dist0,dist1,dist2,dist3
```

- x, y: tag position in metres
- dist0-3: calibrated distance to each base station in metres
