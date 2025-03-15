# BokodeVision
Gaze-Activated Display for Smart Glasses

## Overview
BokodeVision is a computational imaging project that enables gaze-based interaction with physical LEDs. The system uses a Pupil Core eye tracker to detect where a user is looking and activates corresponding LEDs connected to an Arduino microcontroller, creating a hands-free visual interface.

## Features
- Real-time eye gaze tracking using Pupil Core
- Multi-point calibration with statistical modeling
- Advanced filtering techniques for stable detection:
  - Saccade filtering to ignore rapid eye movements
  - Temporal smoothing for stable detection
  - Hysteresis state machine to prevent flickering
  - Minimum fixation duration to avoid accidental activations
- Serial communication with Arduino to control NeoPixel LEDs

## Hardware Requirements
- [Pupil Core](https://pupil-labs.com/products/core/) eye tracker
- Arduino board (e.g., Arduino Uno)
- NeoPixel LEDs (connected to pins 5, 6, and 7)
- Computer with USB ports for both devices

## Software Requirements
- Pupil Service (for eye tracking)
- Python 3.8+ with dependencies (see `requirements.txt`)
- Arduino IDE (for uploading the NeoPixel code)

## Installation
1. Install Python dependencies:
   ```
   pip install -r requirements.txt
   ```

2. Install and run [Pupil Service](https://github.com/pupil-labs/pupil/releases)

3. Upload the `neopixel_activation.cc` code to your Arduino using the Arduino IDE

## Usage
1. Connect your Pupil Core eye tracker and start Pupil Service
2. Connect your Arduino to the specified COM port (default: COM7)
3. Run the main script:
   ```
   python gaze_tracking_and_communication.py
   ```
4. Follow the calibration prompts, looking at each LED when instructed
5. After calibration, the system will enter detection mode, activating LEDs based on your gaze

## Testing
A simple serial communication test script is included:
```
python serial_communication_test.py
```
This allows you to manually control the LEDs by entering commands (0, 1, 2, or 3). x to switch off all LEDs.

## Project Structure
- `gaze_tracking_and_communication.py`: Main Python script for eye tracking and LED control
- `neopixel_activation.cc`: Arduino code for controlling NeoPixel LEDs
- `serial_communication_test.py`: Test script for Arduino communication
- `requirements.txt`: Python dependencies
- `optical_housing.blend`: 3D model of the optical housing. Export as STL for 3D printing. Before exporting, make sure to merge all the parts (apply all transforms) and delete any hidden parts.

## License
This project was developed for the CS448I/EE367 Computational Imaging course at Stanford University.

## Acknowledgments
- Pupil Labs for the eye tracking hardware and software
- Adafruit for the NeoPixel library

