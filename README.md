## ESPuppy Project by:
Tasneem Abu Raya
Nairooz Slaibi
Bisan Jabareen

## Details about the project

 ESPuppy is an interactive robotic dog based on an ESP32 microcontroller.
The system explores human–robot interaction through touch-based input
(head and tail sensors) and emotional responses expressed via:

- Motor movement
- Tail and ear servo motion
- Animated eyes on dual OLED displays
- Audio feedback
- Optional cloud logging via Adafruit IO

The robot operates as a finite state machine (FSM) with multiple emotional
states that are triggered by user interaction and time-based events.

## Folder description :
ESP32:
Source code for the ESP32 side (firmware), including the main application
code that controls sensors, motors, servos, displays, audio, and state logic.

Documentation:
Project documentation files, including:
- USAGE.md – General usage instructions and emotional states
- DEBUG_AND_CALIBRATION.md – Debug messages, error handling, and calibration
- CLOUD.md – Cloud session logging and offline behavior
- Wiring_Diagram.png – Final connection diagram

Unit Tests:
Test sketches used to validate individual hardware components:

- forward_backward_wheels  
  Test for forward and backward DC motor movement

- OLED_SH1106G_displays_test  
  Test for OLED display initialization and rendering

- radio_speaker_test  
  Test for speaker output and audio playback

- sd_single_sound_test  
  Test for SD card access and single MP3 playback

- servo_sweep_test  
  Servo sweep test for angle range and stability

- threshold_test  
  Touch sensor threshold calibration and validation

ASSETS:
Project assets used by the system, including:
- Audio (MP3) files used by the robot


Parameters:
Contains documentation describing hard-coded parameters and configurable
settings used in the project code.


## Arduino/ESP32 libraries used in this project:
- Adafruit SH110X – OLED display control
- Adafruit GFX Library – Graphics primitives
- ESP8266Audio – MP3 decoding and audio playback
- ESP32Servo – Servo motor control (used for testing)
- L298N – DC motor driver control
- Adafruit IO Arduino – Cloud communication
- WiFi (ESP32 core)
- FreeRTOS (ESP32 built-in)


## Connection diagram:
A complete and final connection diagram is provided according to the
project guidelines.

Location:
Documentation/Wiring_Diagram.png

The diagram includes:
- ESP32 GPIO pin assignments
- External 5V power supplies for motors and servos (×2)
- I2C connections for OLED displays
- I2S connections for audio output
- Shared ground between all components

(All libraries were installed using the Arduino Library Manager.)

## Project Poster:
 

