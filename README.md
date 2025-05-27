# YD-LiDAR-Based-Obstacle-Detection-Direction-Control-for-Autonomous-Vehicles
Welcome to the repository of our custom-developed YD-LiDAR based algorithm for autonomous driving systems. This system integrates obstacle detection and directional decision-making using the YD-LiDAR X2/X2L sensor, making it suitable for embedded vehicle control in autonomous navigation tasks.
# YD-LiDAR Based Obstacle Detection & Direction Control for Autonomous Vehicles
![image](https://github.com/user-attachments/assets/5df06123-b22e-46bf-b855-6e56a9b8f16b)

Welcome to the repository of our custom-developed YD-LiDAR based algorithm for autonomous driving systems. This system integrates obstacle detection and directional decision-making using the YD-LiDAR X2/X2L sensor, making it suitable for embedded vehicle control in autonomous navigation tasks.
![image](https://github.com/user-attachments/assets/fb364521-b3db-4a79-96b0-7814598e1b5a)
![image](https://github.com/user-attachments/assets/4633241f-481d-4993-95c6-e894bd2102fb)

## 🚗 Project Overview

In the field of autonomous driving, this project marks a significant innovation. We designed and implemented a **custom decision-making algorithm** to guide autonomous vehicles based on LiDAR scan data. The algorithm detects open space in real-time and commands the vehicle to move in the optimal direction — Forward, Backward, Left, or Right.
![image](https://github.com/user-attachments/assets/e09b7832-700f-45bc-a54c-749bd5b8f0ec)

## ✨ Features
![image](https://github.com/user-attachments/assets/7bfa3e1b-9e46-455f-a3e3-d8fcad453915)

- Real-time LiDAR scan data acquisition using YD-LiDAR SDK
- Automatic port detection and baud rate selection
- Obstacle detection using filtered point cloud data
- Directional decision-making based on maximum distance
- Serial communication-based movement commands (`w`, `a`, `s`, `d`)
- Noise filtering using custom tail-weak strategy
![image](https://github.com/user-attachments/assets/6630e56f-76d8-4e4e-87d5-0089a4c7c814)

## 🧠 Algorithm Highlights
![image](https://github.com/user-attachments/assets/ade1d52a-8719-495a-9c05-c793301cd70b)
![image](https://github.com/user-attachments/assets/c06b2f7c-221f-4988-bf38-ce873bc8593f)

- **Noise Filtering**: Uses tail-weak filtering to clean raw point cloud data.
- **Max Distance Evaluation**: Determines the direction with the maximum available space.
- **Command Triggering**: Sends serial commands to the vehicle for motion control.
- **Dynamic Port and Baud Rate Selection**: Enhances usability and portability.
![image](https://github.com/user-attachments/assets/1a3a7816-416c-47e9-a1d6-6e110765d74c)

## 🖥️ Code Structure
![image](https://github.com/user-attachments/assets/c32ad27f-72d2-46f2-b9db-65990a188e14)

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
![image](https://github.com/user-attachments/assets/ca79bf47-c97c-4d4c-919b-80ff908a1fc6)
![image](https://github.com/user-attachments/assets/20049101-7e05-436f-9073-83180ca666a6)

## 🛠 Serial Commands Mapping

| Command | Action    |
|---------|-----------|
| `w`     | Forward   |
| `s`     | Backward  |
| `a`     | Left      |
| `d`     | Right     |
| `x`     | Stop      |
![image](https://github.com/user-attachments/assets/fdf5b946-7786-4dbf-a01a-7fb2bccffd67)

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
