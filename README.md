# radioROS

**radioROS** is a simple Python-based ROS2 package that allows robots to utilize an RC receiver in their projects. This package should theoretically work for any receiver since there is an ESP32 that carefully reads the PWM data and publishes it via serial. Other packages use SBUS, which is not standardized, and most receivers don't have this feature, and those that do are expensive.

![image](https://github.com/user-attachments/assets/2481bc46-0bf0-4c46-ac5b-8a36afa3a52b)

![image](https://github.com/user-attachments/assets/316cd695-f004-469e-a912-12977a2e2801)

## Getting Started

### Hardware Setup

1. First, get an ESP32 and upload the `esp32_code.ino` that's within the `firmware` directory.
2. Connect the ESP32 to the receiver as follows:
   
   ![image](https://github.com/user-attachments/assets/5c478eb2-cfaa-4c08-a674-800af1c13867)

   It's very simple with only 8 connections in total. Be sure to pair the receiver beforehand.

### Software Setup

1. Clone this repo into your ROS2 workspace:

    ```sh
    cd ros2_ws/src
    git clone https://github.com/TheHassanShahzad/radioROS.git
    cd ..
    colcon build --packages-select radioROS
    source install/setup.bash
    ```

## Nodes

The package contains two nodes:

1. **pwm_publisher**
   - Responsible for reading the incoming serial data from the ESP32 and publishing the data to the `/receiver_data` topic of type `Int16MultiArray`.
   - Takes `serial_port` as a parameter (default is `/dev/ttyUSB0`).

2. **pwm_to_twist**
   - Subscribes to the `/receiver_data` topic and publishes `Twist` messages to the `/cmd_vel` topic.
   - Takes the following parameters:
     - `max_linear_speed` (default is 1.0)
     - `max_angular_speed` (default is 1.0)
     - `topic` (default is `cmd_vel`)

### Important Notes

- `pwm_to_twist` can only work if `pwm_publisher` is running.
- `pwm_publisher` only works if the transmitter is powered on, so the ESP32 is publishing PWM data.

## Firmware for ESP32

The ESP32 code required to make this ROS2 package work can be found in the `firmware` directory. Follow these steps to upload the code to your ESP32:

1. Open the `esp32_code.ino` file located in the `firmware` directory using the Arduino IDE or any other compatible IDE.
2. Connect your ESP32 to your computer.
3. Select the correct board and port in the IDE.
4. Upload the code to the ESP32.
