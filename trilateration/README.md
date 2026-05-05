# UWB Indoor Trilateration

2D indoor positioning using 4x BU-04 UWB modules and an Arduino Mega.

## Hardware

- 4x BU-04 anchor modules, each with an Arduino Nano ESP32 and battery
- 1x BU-04 tag module connected to Arduino Mega
- 1x Arduino Mega running the trilateration code

## Wiring

**Anchors (Arduino Nano ESP32):**
- BU-04 TX1 → D2 (GPIO5) — config UART
- BU-04 RX1 → D3 (GPIO6) — config UART
- BU-04 TX2 → RX0 (GPIO44) — data UART
- BU-04 RX2 → TX0 (GPIO43) — data UART
- BU-04 3V3 → 3.3V
- BU-04 GND → GND

**Tag (Arduino Mega):**
- BU-04 TX2 → Mega pin 17 (RX2)
- BU-04 RX2 → Mega pin 16 (TX2)
- BU-04 3V3 → 3.3V
- BU-04 GND → GND

## Anchor Configuration

Flash each anchor with the configure sketch, setting the correct ID:

```cpp
configureBU04(0, 1, 1, 1); // BS0
configureBU04(1, 1, 1, 1); // BS1
configureBU04(2, 1, 1, 1); // BS2
configureBU04(3, 1, 1, 1); // BS3
```

Tag:

```cpp
configureBU04(0, 0, 1, 1);
```

## Physical Placement

Place anchors at the four corners of the room/container:
