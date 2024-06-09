# Clocky

Clocky is an ESP32-based digital clock project that includes features such as displaying time, date, temperature, humidity, and controlling display modes.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Requirements](#requirements)
- [Setup](#setup)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

Clocky is a customizable digital clock project designed for ESP32 microcontrollers. It uses various sensors and libraries to provide a range of functionalities, making it suitable for home automation, IoT projects, or personal use.

## Features

- Display time in 12-hour or 24-hour format.
- Show current date, temperature, and humidity.
- Has a built in buzzer for the alarm.
- Light sensor for dimming the display automatically when dark. 
- Multiple display modes including scrolling text and animations.
- Infrared remote control for convenient operation.
- Dismiss the alarm with a wave.
- Choose to only show the time etc. momentarily with a wave.
- Integration with external sensors and modules.

## Requirements

To use Clocky, you will need the following:

- ESP32 development board.
- Sensors such as SHT31, DHT11/DHT22 for temperature and humidity sensing.
- LED matrix display compatible with the MD_MAX72XX library.
- Arduino IDE for development.
- Necessary libraries (see the "Installed Libraries" file for details).

## Setup

1. Clone or download the Clocky repository to your local machine.
2. Install the required libraries using Arduino Library Manager.
3. Connect your ESP32 board and sensors according to the wiring diagram provided in the project.
4. Open the project in Arduino IDE.
5. Upload the code to your ESP32 board.

## Usage

1. Power on your Clocky device.
2. The clock will start displaying the time and other information based on the selected mode.
3. Use the Infrared remote control buttons to switch between modes or configure settings.
4. Customize the code to add new features or modify existing ones according to your requirements.

## Contributing

Contributions to Clocky are welcome! If you have ideas for improvements, new features, or bug fixes, feel free to fork the repository and submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE), which means you can use, modify, and distribute the code for personal or commercial purposes. See the `LICENSE` file for more details.
