/*
  Copyright(c) 2021:
  - Huang Chenrui <hcr2077@outlook.com>

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#ifndef STARNETO_MEMS_HPP
#define STARNETO_MEMS_HPP

#include "starneto_ros_msgs/Gpfpd.h"
#include "starneto_ros_msgs/Gtimu.h"
#include "starneto_ros_msgs/Pos320Nav.h"
#include "serial/serial.h"

namespace ns_starneto_mems {

class Starneto {

 public:

  // Constructor
  Starneto(ros::NodeHandle &nodeHandle);
  int getNodeRate() const;

  // Method
  void loadParameters();
  void publishToTopics();
  void initSerial();
  void run();
  void initState();
  void readSerial();
  int  analyzeProtocol();
  void analyzeGpfpd();
  void analyzeGtimu();
  void sendMsg();
  void runAlgorithm();
 
 private:
  ros::NodeHandle nodeHandle;
  int node_rate;
  std::string serial_port;
  std::string GPFPD_output_topic;
  std::string GTIMU_output_topic;

  char OneFrame[200];   // one frame data
  unsigned char rbuf[1000];  // 接收缓冲区
  int numinbuf;     // buffer num
  int numgetted;    // num get
  int protocolFlag; // 0:Unknown 1:GPFPD 2:GTIMU
  int CntDelimiter;//分隔符计数
  int PosDelimiter[15];//用于记录分隔符位置
  int field_len[15];//字符串长度
  char temp_field[30];
  char str[3];
  unsigned int tmpint;
  int cscomputed;//计算得到的校验，除去$*hh<CR><LF>共6个字符
  int csreceived;//接收到的校验
  char strtemp[3];
  double delta_t;
  double gpstime_pre;//上一个gps时间

  int GPFPD_STATE_PARSER;
  int GTIMU_STATE_PARSER;

  const int UNKNOWN_PROTOCOL = 0;
  const int GPFPD_ENABLE = 1;
  const int GTIMU_ENABLE = 2;

  serial::Serial ser;
  starneto_ros_msgs::Gpfpd gnss;
  starneto_ros_msgs::Gtimu imu;
  // Publisher
  ros::Publisher pub_gpfpd;
  ros::Publisher pub_gtimu;
};

 
}
  


  


#endif //STARNETO_MEMS_HPP
