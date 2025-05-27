/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include "CYdLidar.h"
#include "filters/NoiseFilter.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif
//------------------------------------------------------

int openSerialPort() {
    int serialPort = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (serialPort == -1) {
        cerr << "Error: Could not open serial port." << endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serialPort, &tty) != 0) {
        cerr << "Error: Could not get terminal attributes." << endl;
        return -1;
    }

    cfsetospeed(&tty, B115200);  // Set output baud rate
    cfsetispeed(&tty, B115200);  // Set input baud rate

    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver and local mode
    tty.c_cflag &= ~CSIZE;          // Clear character size mask
    tty.c_cflag |= CS8;             // Set 8 data bits
    tty.c_cflag &= ~CSTOPB;         // 1 stop bit
    tty.c_cflag &= ~PARODD;         // No parity
    tty.c_cflag &= ~PARENB;         // No parity

    tty.c_lflag &= ~ICANON;         // Disable canonical mode (no buffering)
    tty.c_lflag &= ~ECHO;           // Disable echo
    tty.c_lflag &= ~ECHOE;          // Disable erasure
    tty.c_lflag &= ~ECHONL;         // Disable new-line echo
    tty.c_lflag &= ~ISIG;           // Disable signal chars like Ctrl-C

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Disable line processing

    tty.c_oflag &= ~OPOST;          // Disable output processing

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        cerr << "Error: Could not set terminal attributes." << endl;
        return -1;
    }

    return serialPort;
}

//------------------------------------------------------
void sendCommand(int serialPort, char command) {
    write(serialPort, &command, 1);
    cout << "Command sent: " << command << endl;
}


/**
 * @brief tof Lidar test
 * @param argc
 * @param argv
 * @return
 * @par Flow chart
 * Step1: instance CYdLidar.\n
 * Step2: set paramters.\n
 * Step3: initialize SDK and LiDAR.(::CYdLidar::initialize)\n
 * Step4: Start the device scanning routine which runs on a separate thread and enable motor.(::CYdLidar::turnOn)\n
 * Step5: Get the LiDAR Scan Data.(::CYdLidar::doProcessSimple)\n
 * Step6: Stop the device scanning thread and disable motor.(::CYdLidar::turnOff)\n
 * Step7: Uninitialize the SDK and Disconnect the LiDAR.(::CYdLidar::disconnecting)\n
 */
int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  ydlidar::os_init();

  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;//

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::os_isOk()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  int baudrate = 230400;
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 230400;
  baudrateList[2] = 460800;
  baudrateList[3] = 512000;

  printf("Baudrate:\n");

  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++) {
    printf("%d. %d\n", it->first, it->second);
  }

  while (ydlidar::os_isOk()) {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }

  bool isSingleChannel = false;
  std::string input_channel;
  printf("Whether the Lidar is one-way communication[yes/no]:");
  std::cin >> input_channel;
  std::transform(input_channel.begin(), input_channel.end(),
                 input_channel.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_channel.find("y") != std::string::npos) {
    isSingleChannel = true;
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }

  std::string input_frequency;

  float frequency = 8.0f;

  while (ydlidar::os_isOk() && !isSingleChannel) {
    printf("Please enter the lidar scan frequency[3-15.7]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());

    if (frequency <= 15.7 && frequency >= 3.0) {
      break;
    }

    fprintf(stderr,
            "Invalid scan frequency,The scanning frequency range is 3 to 15.7 HZ, Please re-enter.\n");
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }



  /// instance
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  /// tof lidar
  int optval = TYPE_TOF;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = isSingleChannel ? 4 : 20;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
//  optval = 16;
//  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = true;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = false;
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = false;
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = isSingleChannel;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: Â°
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  /// initialize SDK and LiDAR.
  bool ret = laser.initialize();

  if (ret) {//success
    /// Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  } else {//failed
    fprintf(stderr, "%s\n", laser.DescribeError());
    fflush(stderr);
  }

  LaserScan scan;
  LaserScan outScan;
  NoiseFilter filter;
  filter.setStrategy(NoiseFilter::FS_TailWeek);

 
    /// Turn On success and loop
    if (laser.doProcessSimple(scan))
    {
      fprintf(stdout, "Scan received at [%f]Hz %u points is [%f]s\n",
              scan.scanFreq,
              (unsigned int)scan.points.size(),
              scan.config.scan_time);

      // ä½¿ç”¨æ‹–å°¾æ»¤æ³¢å™¨
//----------------------------------------------------------------
	float max_distance = 0;
	float max_angle = 0;
        filter.filter(scan, 0, 0, outScan);
       for (size_t i = 0; i < scan.points.size(); ++i)
       {
         const LaserPoint &p = scan.points.at(i);
         printf("%d a %.02f d %.03f i %.0f\n",
                i, p.angle * 180 / M_PI, p.range, p.intensity);
 if (p.range > max_distance) {
        max_distance = p.range;
        max_angle = p.angle * 180 / M_PI; // Convert angle to degrees for output
    }
       }
//-----------------------------------------------------------------

bool forward = false, backward = false, left = false, right = false;
// Determine which side the largest distance angle belongs to
std::string direction;
if (max_angle >= 45 && max_angle <= 135) {
    direction = "Forward";
	forward = true;
} else if ((max_angle > 135 && max_angle <= 180) || (max_angle >= -180 && max_angle < -135)) {
    direction = "Right";
	right = true;
} else if (max_angle >= -135 && max_angle <= -45) {
    direction = "Backward";
	backward = true;
} else if (max_angle > -45 && max_angle < 45) {
    direction = "Left";
 	left = true;
}

printf("Largest distance: %.03f at angle: %.02f, Direction: %s\n", max_distance, max_angle, direction.c_str());
//-------------------------------------------------------------

 // Send command based on direction
            if (forward) sendCommand(serialPort, 'w');
            else if (backward) sendCommand(serialPort, 's');
            else if (left) sendCommand(serialPort, 'a');
            else if (right) sendCommand(serialPort, 'd');
            else sendCommand(serialPort, 'x'); // stop



      fflush(stdout);
    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  

  /// Stop the device scanning thread and disable motor.
  laser.turnOff();
  close(serialPort); 
  /// Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();

  return 0;
}
