# EasyRC - open-source RC platform

![EasyRC vehicle](https://github.com/user-attachments/assets/3aadc515-c354-4103-b193-75088adee5f9)

Description
---
EasyRC is an easy-to-build platform for creating your own remote-controlled vehicle. The platform includes CAD files for printing an affordable 1:16 scale car as well as a small remote control. Further, it includes the project and production files for the printed circuit boards that are needed to power these devices. Last but not least, it lists all required components in a bill of materials and comes with a complete instruction manual that shows how to print, solder, assemble, program, and adjust everything for running the car. The legacy folder contains data and information from previous revisions of the project.

The idea of this platform is to serve as a starting point for people who are interested in DIY hobby electronics as well as 3D printing, but were reluctant so far due to the initial burdens that are associated with the complexity of these topics. Hence, EasyRC offers a simple start for beginners, but due to its open-source character, it offers full freedom to tweak, redesign, and reprogram all parts of the project.

The 1:16 scale vehicle is designed for indoor use (for the sake of simplicity, it does not encompass suspension) and can be safely operated by kids (maximum speed is 10 km/h but can be set lower within the software). Hence, the project is also a great gift for kids who want to dive into engineering and programming topics, or start with driving a selfmade remote-controlled car and look behind the scenes once they're getting older.

The manual is available in English and German.

Specifications
---
Vehicle:
- Size: 282x125x95 mm³ (LxWxH)
- Scale: 1:16
- Weight: approx. 1 kg (including batteries)
- Microcontroller: ESP32-C3
- Wireless communication: 2.4 GHz (ESP-NOW protocol)
- Motor driver: Pololu DRV8876 (QFN)
- Drive motor: JGA25-370 Brushed DC motor with gearbox (12 V, 1000 U/min)
- Speed: max. 10 km/h
- Steering servo: MG90S metal-gear micro servo
- Power supply: 3x 18650 Li-Ion battery (11.1 V)
- Power consumption: approx. 0,8 W (idle); approx. 3-6 W (driving)

Remote control:
- Size: 80x40x41 mm³ (LxWxH)
- Weight: approx. 125 g (including battery)
- Microcontroller: ESP32-C3
- Wireless communication: 2.4 GHz (ESP-NOW protocol)
- Joystick: Alps 10k (Playstation 4 model)
- Power supply: 1x 18650 Li-Ion battery (3.7 V)
- Power consumption: approx. 0.5 W

Safety information:
---
- Voltage levels are 12 V or less (harmless for humans)
-	Battery connectors are reverse polarity protected
-	Low battery warnings are included
-	The motor automatically stops if the connection to the remote control is lost


