# YD-LiDAR-Based-Obstacle-Detection-Direction-Control-for-Autonomous-Vehicles
Welcome to the repository of our custom-developed YD-LiDAR based algorithm for autonomous driving systems. This system integrates obstacle detection and directional decision-making using the YD-LiDAR X2/X2L sensor, making it suitable for embedded vehicle control in autonomous navigation tasks.
# YD-LiDAR Based Obstacle Detection & Direction Control for Autonomous Vehicles

Welcome to the repository of our custom-developed YD-LiDAR based algorithm for autonomous driving systems. This system integrates obstacle detection and directional decision-making using the YD-LiDAR X2/X2L sensor, making it suitable for embedded vehicle control in autonomous navigation tasks.

## 🚗 Project Overview

In the field of autonomous driving, this project marks a significant innovation. We designed and implemented a **custom decision-making algorithm** to guide autonomous vehicles based on LiDAR scan data. The algorithm detects open space in real-time and commands the vehicle to move in the optimal direction — Forward, Backward, Left, or Right.

## ✨ Features

- Real-time LiDAR scan data acquisition using YD-LiDAR SDK
- Automatic port detection and baud rate selection
- Obstacle detection using filtered point cloud data
- Directional decision-making based on maximum distance
- Serial communication-based movement commands (`w`, `a`, `s`, `d`)
- Noise filtering using custom tail-weak strategy

## 🧠 Algorithm Highlights

- **Noise Filtering**: Uses tail-weak filtering to clean raw point cloud data.
- **Max Distance Evaluation**: Determines the direction with the maximum available space.
- **Command Triggering**: Sends serial commands to the vehicle for motion control.
- **Dynamic Port and Baud Rate Selection**: Enhances usability and portability.

## 🖥️ Code Structure

- `lidar.cpp`: Main source file containing logic for initializing LiDAR, processing scan data, determining direction, and controlling vehicle.
- Uses:
  - `CYdLidar` class from YD-LiDAR SDK
  - `NoiseFilter` for filtering invalid data points
  - Standard C++ I/O and system libraries

## 🔧 Dependencies

- C++11 or later
- [YD-LiDAR SDK](https://github.com/YDLIDAR/YDLidar-SDK)
- Compatible LiDAR model: YDLIDAR X2 / X2L
- Linux environment (Tested on Ubuntu)

## 🚀 How to Run

1. **Connect the YD-LiDAR to your system**
2. **Compile the code**:

    ```bash
    g++ lidar.cpp -o lidar_app -I/path/to/ydlidar/include -L/path/to/ydlidar/lib -lydlidar_sdk
    ```

3. **Run the application**:

    ```bash
    ./lidar_app
    ```

4. **Follow on-screen prompts** to select port, baudrate, and communication type.

## 🛠 Serial Commands Mapping

| Command | Action    |
|---------|-----------|
| `w`     | Forward   |
| `s`     | Backward  |
| `a`     | Left      |
| `d`     | Right     |
| `x`     | Stop      |

## 👨‍💻 Authors

- **Soma Sekhar Pavuluri** – Lead Developer & Researcher  
- Special thanks to the team for contributions to LiDAR integration and algorithm tuning.

## 📜 License

This project uses the BSD License under the YDLIDAR SDK. See `lidar.cpp` for full license terms.

## 💡 Future Work

- Integration with SLAM for map-aware navigation
- Real-time GUI-based obstacle visualization
- Adaptive filtering and AI-based decision-making

---

> 🚘 *"Driving the future of autonomy through smarter sensing!"*
