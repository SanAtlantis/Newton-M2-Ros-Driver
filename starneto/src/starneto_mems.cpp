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

#include <ros/ros.h>
#include "starneto_mems.hpp"
#include <sstream>

namespace ns_starneto_mems {

    void Starneto::run() {
        //  Here to start
        if (!ser.isOpen()) {
            ROS_ERROR_STREAM("serial port: " << serial_port << " init failed.");
            return;
        }
        Starneto::initState();
        Starneto::readSerial();
    }

    void Starneto::initState() {
        numinbuf = 0;
        numgetted = 0;
        protocolFlag = UNKNOWN_PROTOCOL;
        memset(OneFrame, 0, sizeof(OneFrame));
        memset(rbuf, 0, sizeof(rbuf));
    }

    void Starneto::readSerial() {
        try {
            numinbuf = ser.available();//available()返回从串口缓冲区读回的字符数
            //printf("bytes in buf = %d\n", numinbuf);
        }
        catch (serial::IOException &e) {
            ROS_ERROR_STREAM("Port crashed！ Please check cable!");
        }
        if (numinbuf > 0) {
            numgetted = ser.read(rbuf, numinbuf);//串口缓冲区数据读到rbuf中
            // 取回的数据个数与缓冲区中有的数据个数相同，说明读串口成功
            // printf("RBUF $ is: %s\n",rbuf);//输出接收到$前收到的数据
            if (numgetted == numinbuf) {
                Starneto::runAlgorithm();
            }
        }// if (numinbuf > 0)
    }

    /*****************************
  功能：计算校验，字符串中所有字符的异或
  返回：返回一个无符号整数
  输入参数：<1>字符串指针，<2>字符串长度（指有效长度，不包括字符串结束标志）
  输出参数：校验结果
******************************/
    unsigned int GetXorChecksum(const char *pch, int len) {
        unsigned int cs = 0;
        int i;

        for (i = 0; i < len; i++)
            cs ^= pch[i];

        return cs;
    }

    /********************************
	功能：度分格式转换为度GPFPD
********************************/
    double DegMin2Deg(double dddmmpmmmm) {
        double ddd = floor(dddmmpmmmm / 100.0);
        double mmpmmmm = dddmmpmmmm - ddd * 100.0;
        return ddd + mmpmmmm / 60.0;
    }

    void Starneto::runAlgorithm() {
        int CntByte = 0;
        int select_flag = 0;
        CntDelimiter = 0; //分隔符计数
        PosDelimiter[15] = {0}; //用于记录分隔符位置
        field_len[15] = {0}; //字符串长度
        temp_field[30] = {0}; // temp char for calc data
        str[3] = {0};
        tmpint = 0;

        for (int i = 0; i < numgetted; i++) {
            if (protocolFlag == UNKNOWN_PROTOCOL) {
                if (rbuf[i] == '$') { select_flag = 1; }
                if (select_flag) {
                    OneFrame[CntByte] = rbuf[i];
                    CntByte++;
                    //printf("%d", CntByte);
                }
                if (select_flag && rbuf[i] == ',') {
                    if (strncmp(OneFrame, "$GPFPD,", 7) == 0) {
                        protocolFlag = GPFPD_ENABLE;
                        GPFPD_STATE_PARSER = 2;
                        CntDelimiter = 0;//分隔符计数从0开始
                        PosDelimiter[0] = CntByte - 1;//记录分隔符在OneFrame中的位置
                    } else if (strncmp(OneFrame, "$GTIMU,", 7) == 0) {
                        protocolFlag = GTIMU_ENABLE;
                        GTIMU_STATE_PARSER = 2;
                        CntDelimiter = 0;//分隔符计数从0开始
                        PosDelimiter[0] = CntByte - 1;//记录分隔符在OneFrame中的位置
                    } else {
                        protocolFlag = UNKNOWN_PROTOCOL;
                        ROS_ERROR_STREAM("UNKNOWN_PROTOCOL.");
                    }
                    select_flag = 0;
                    continue; // Don't Delete It Or Oneframe is wrong
                }
            }// protocol unknown
/**************************************************************
 ************************ GPFPD *******************************
 **************************************************************/
            if (protocolFlag == GPFPD_ENABLE) {
                switch (GPFPD_STATE_PARSER) {
                    case 2:
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;
                        if (rbuf[i] == ',') {
                            CntDelimiter++;//分隔符计数
                            PosDelimiter[CntDelimiter] = CntByte - 1;//记下分隔符位置
                            //0-14，共15个分隔符，开始数据解析
                            if (CntDelimiter == 14) {
                                //printf("One frame: %s\n",OneFrame); // test Pass
                                for (int j = 1; j <= 14; j++) {
                                    field_len[j - 1] = PosDelimiter[j] - PosDelimiter[j - 1] - 1;//数据域长度
                                }
                                //TODO: TEST PASS !
                                if (field_len[0] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[0] + 1], field_len[0]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.gpsweek = atoi(temp_field);
                                    //printf("gpsweek=%d\n", msg_xwgi5651_gpfpd.gpsweek);
                                }

                                if (field_len[1] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[1] + 1], field_len[1]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.gpstime = atof(temp_field);
                                    //printf("gpstime=%lf\n", msg_xwgi5651_gpfpd.gpstime);
                                }

                                if (field_len[2] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[2] + 1], field_len[2]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.heading = atof(temp_field);
                                    //TODO: Change to car frame
                                    if (gnss.heading > 180.0) {
                                        gnss.heading = 360 - gnss.heading;
                                    } else {
                                        gnss.heading = -gnss.heading;
                                    }
                                    //printf("heading=%lf\n", msg_xwgi5651_gpfpd.heading);
                                }

                                if (field_len[3] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[3] + 1], field_len[3]);
                                    gnss.pitch = atof(temp_field);
                                    //TODO: Change to car frame
                                    gnss.pitch = -gnss.pitch;
                                }

                                if (field_len[4] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[4] + 1], field_len[4]);
                                    //TODO: Change to car frame
                                    gnss.roll = atof(temp_field);
                                    //printf("roll=%lf\n", msg_xwgi5651_gpfpd.roll);
                                }

                                if (field_len[5] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[5] + 1], field_len[5]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.lat = atof(temp_field);
                                    //printf("lat=%15.10lf\n", msg_xwgi5651_gpfpd.lat);
                                }

                                if (field_len[6] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[6] + 1], field_len[6]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.lon = atof(temp_field);
                                    //printf("lon=%15.10lf\n", msg_xwgi5651_gpfpd.lon);
                                }

                                if (field_len[7] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[7] + 1], field_len[7]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.alt = atof(temp_field);
                                    //printf("alt=%lf\n", msg_xwgi5651_gpfpd.alt);
                                }

                                if (field_len[8] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[8] + 1], field_len[8]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.ve = atof(temp_field);
                                    //printf("ve=%lf\n", msg_xwgi5651_gpfpd.ve);
                                }

                                if (field_len[9] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[9] + 1], field_len[9]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.vn = atof(temp_field);
                                    //printf("vn=%lf\n", msg_xwgi5651_gpfpd.vn);
                                }

                                if (field_len[10] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[10] + 1], field_len[10]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.vu = atof(temp_field);
                                    //printf("vu=%lf\n", msg_xwgi5651_gpfpd.vu);
                                }

                                if (field_len[11] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[11] + 1], field_len[11]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.baseline = atof(temp_field);
                                    //printf("baseline=%lf\n", msg_xwgi5651_gpfpd.baseline);
                                }

                                //nsv1
                                if (field_len[12] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[12] + 1], field_len[12]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.nsv1 = atoi(temp_field);
                                    //printf("nsv1=%d\n", msg_xwgi5651_gpfpd.nsv1);
                                }

                                //nsv2
                                if (field_len[13] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[13] + 1], field_len[13]);
                                    //printf("temp_field=%s\n",temp_field);
                                    gnss.nsv2 = atoi(temp_field);
                                    //printf("nsv2=%d\n", msg_xwgi5651_gpfpd.nsv2);
                                }
                                GPFPD_STATE_PARSER = 3;
                            }// if CntDelimiter == 14
                        }// if rbuf[i] == ','
                        break;
                    case 3: //等待接收状态字节，第一个字符
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;//指向下一个空位
                        if ((rbuf[i] >= '0' && rbuf[i] <= '9') || (rbuf[i] >= 'A' && rbuf[i] <= 'F'))//状态字节应是一个十六进制数
                        {
                            GPFPD_STATE_PARSER = 4;
                        } else {
                            memset(OneFrame, 0, sizeof(OneFrame));
                            GPFPD_STATE_PARSER = 0;
                            ROS_ERROR_STREAM("DATA ERROR IN CASE3.");
                        }
                        break;
                    case 4: //等待接收状态字节，第二个字符
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;//指向下一个空位
                        //printf("status=0x%c%c\n",OneFrame[CntByte-2],OneFrame[CntByte-1]);
                        if ((rbuf[i] >= '0' && rbuf[i] <= '9') || (rbuf[i] >= 'A' && rbuf[i] <= 'F'))//状态字节应是一个十六进制数
                        {
                            GPFPD_STATE_PARSER = 5;
                        } else {
                            memset(OneFrame, 0, sizeof(OneFrame));
                            GPFPD_STATE_PARSER = 0;
                            ROS_ERROR_STREAM("DATA ERROR IN CASE4.");
                        }
                        str[0] = OneFrame[CntByte - 2];
                        str[1] = OneFrame[CntByte - 1];
                        str[2] = '\0';//字符串结束标志
                        sscanf(str, "%x", &tmpint);//字符串str按16进制数转换为整数
                        gnss.status = tmpint;
                        break;
                    case 5: //等待校验和标志*
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;//指向下一个空位
                        if (rbuf[i] == '*') {
                            GPFPD_STATE_PARSER = 6;
                        } else {
                            memset(OneFrame, 0, sizeof(OneFrame));
                            GPFPD_STATE_PARSER = 0;
                            ROS_ERROR_STREAM("DATA ERROR IN CASE5.");
                        }
                        break;
                    case 6: //校验和第一个字符
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;//指向下一个空位
                        if ((rbuf[i] >= '0' && rbuf[i] <= '9') || (rbuf[i] >= 'A' && rbuf[i] <= 'F'))//校验和字节应是一个十六进制数
                        {
                            GPFPD_STATE_PARSER = 7;
                        } else {
                            memset(OneFrame, 0, sizeof(OneFrame));
                            GPFPD_STATE_PARSER = 0;
                            ROS_ERROR_STREAM("DATA ERROR IN CASE6.");
                        }
                        break;
                    case 7: //校验和第二个字符
                        OneFrame[CntByte] = rbuf[i];
                        //printf("case7i %x \n", rbuf[i]);
                        CntByte++;//指向下一个空位
                        if ((rbuf[i] >= '0' && rbuf[i] <= '9') || (rbuf[i] >= 'A' && rbuf[i] <= 'F'))//校验和字节应是一个十六进制数
                        {
                            //检查校验
                            cscomputed = GetXorChecksum((char *) (OneFrame + 1),
                                                        CntByte - 4);//计算得到的校验，除去$*hh<CR><LF>共6个字符
                            csreceived = 0;//接收到的校验
                            strtemp[0] = OneFrame[CntByte - 2];
                            strtemp[1] = OneFrame[CntByte - 1];
                            strtemp[2] = '\0';//字符串结束标志
                            sscanf(strtemp, "%x", &csreceived);//字符串strtemp转换为16进制数
                            //检查校验是否正确
                            if (cscomputed != csreceived)//校验和不匹配
                            {
                                memset(OneFrame, 0, sizeof(OneFrame));
                                GPFPD_STATE_PARSER = 0;
                                ROS_ERROR_STREAM("DATA ERROR IN CASE7.");
                            } else//校验和匹配
                            {
                                GPFPD_STATE_PARSER = 8;
                            }
                        }//校验和字节是hex
                        break;
                    case 8: //等待结束标志<CR>=0x0d
                        OneFrame[CntByte] = rbuf[i];
                        //printf("case8i-1 %x \n",rbuf[i-1]);
                        //printf("case8i+1 %x \n",rbuf[i+1]);
                        CntByte++;//指向下一个空位
                        if (rbuf[i] == '\r') {
                            //printf("End flag <CR>=0x0%x OK!\n",rbuf[i]);
                            GPFPD_STATE_PARSER = 9;
                        } else {
                            //msg_xwgi5651_gpfpd.flag_cr = 1;
                            //PubMsg();
                            memset(OneFrame, 0, sizeof(OneFrame));
                            GPFPD_STATE_PARSER = 0;
                            ROS_ERROR_STREAM("DATA ERROR IN CASE8.");
                            //printf("End flag <CR> error!\n");
                        }
                        break;
                    case 9: //等待结束标志<LF>=0x0a
                        OneFrame[CntByte] = rbuf[i];
                        if (rbuf[i] == '\n') {
                            delta_t = gnss.gpstime - gpstime_pre;//前后两帧之间的时间差
                            gpstime_pre = gnss.gpstime;
                            ROS_INFO_STREAM("GPS DATA OK.");
                            //CLEAR_LINE();
                            //printf("Frame period=%f(sec)\n", delta_t);
                            //CLEAR_LINE();
                            //printf("A good frame!\n");
                            //MOVEUP(3);//光标上移3行
                            //CLEAR();//清屏
                        } else {
                            //printf("End flag <LF> error!\n");
                            ROS_ERROR_STREAM("DATA ERROR IN CASE9.");
                        }
                        memset(OneFrame, 0, sizeof(OneFrame));
                        GPFPD_STATE_PARSER = 0;
                        break; //结束了数据解析 代码写的太耦合了 fxxk
                    default:
                        memset(OneFrame, 0, sizeof(OneFrame));
                        GPFPD_STATE_PARSER = 0;
                        break;
                }// switch
            }// protocol GPFPD
/**************************************************************
 ************************ GTIMU *******************************
 **************************************************************/
            if (protocolFlag == GTIMU_ENABLE) {
                switch (GTIMU_STATE_PARSER) {
                    case 2:
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;
                        // rbuf[i] == '*' to fit special frame
                        if (rbuf[i] == ',' || rbuf[i] == '*') {
                            CntDelimiter++;//分隔符计数
                            PosDelimiter[CntDelimiter] = CntByte - 1;//记下分隔符位置
                            //0-9，共10个分隔符，开始数据解析
                            if (CntDelimiter == 9) {
                                for (int j = 1; j <= 9; j++) {
                                    field_len[j - 1] = PosDelimiter[j] - PosDelimiter[j - 1] - 1;//数据域长度
                                }
                                //TODO: TEST PASS!
                                if (field_len[0] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[0] + 1], field_len[0]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.gpsweek = atoi(temp_field);
                                    //printf("gpsweek=%d\n", imu.gpsweek);
                                }

                                if (field_len[1] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[1] + 1], field_len[1]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.gpstime = atof(temp_field);
                                    //printf("gpstime=%lf\n", imu.gpstime);
                                }

                                if (field_len[2] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[2] + 1], field_len[2]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.gx = atof(temp_field);
                                    //printf("heading=%lf\n", msg_xwgi5651_gpfpd.heading);
                                }

                                if (field_len[3] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[3] + 1], field_len[3]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.gy = atof(temp_field);
                                    //printf("pitch=%lf\n", msg_xwgi5651_gpfpd.pitch);
                                }

                                if (field_len[4] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[4] + 1], field_len[4]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.gz = atof(temp_field);
                                    //printf("roll=%lf\n", msg_xwgi5651_gpfpd.roll);
                                }

                                if (field_len[5] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[5] + 1], field_len[5]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.ax = atof(temp_field);
                                    //printf("lat=%15.10lf\n", msg_xwgi5651_gpfpd.lat);
                                }

                                if (field_len[6] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[6] + 1], field_len[6]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.ay = atof(temp_field);
                                    //printf("lon=%15.10lf\n", msg_xwgi5651_gpfpd.lon);
                                }

                                if (field_len[7] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[7] + 1], field_len[7]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.az = atof(temp_field);
                                    //printf("alt=%lf\n", msg_xwgi5651_gpfpd.alt);
                                }

                                if (field_len[8] > 0) {
                                    memset(temp_field, 0, sizeof(temp_field));
                                    strncpy(temp_field, &OneFrame[PosDelimiter[8] + 1], field_len[8]);
                                    //printf("temp_field=%s\n",temp_field);
                                    imu.tpr = atof(temp_field);
                                    //printf("ve=%lf\n", msg_xwgi5651_gpfpd.ve);
                                }
                                GTIMU_STATE_PARSER = 3;
                            }// if cntdelimiter ==8
                        }// if rbuf[i] == ','
                        break;
                    case 3: //校验和第一个字符
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;//指向下一个空位
                        if ((rbuf[i] >= '0' && rbuf[i] <= '9') || (rbuf[i] >= 'A' && rbuf[i] <= 'F'))//校验和字节应是一个十六进制数
                        {
                            GTIMU_STATE_PARSER = 4;
                        } else {
                            memset(OneFrame, 0, sizeof(OneFrame));
                            GTIMU_STATE_PARSER = 0;
                            ROS_ERROR_STREAM("IMU ERROR IN CASE3.");
                        }
                        break;
                    case 4: //校验和第二个字符
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;//指向下一个空位
                        if ((rbuf[i] >= '0' && rbuf[i] <= '9') || (rbuf[i] >= 'A' && rbuf[i] <= 'F'))//校验和字节应是一个十六进制数
                        {
                            //检查校验
                            cscomputed = GetXorChecksum((char *) (OneFrame + 1),
                                                        CntByte - 4);//计算得到的校验，除去$*hh<CR><LF>共6个字符
                            csreceived = 0;//接收到的校验
                            strtemp[0] = OneFrame[CntByte - 2];
                            strtemp[1] = OneFrame[CntByte - 1];
                            strtemp[2] = '\0';//字符串结束标志
                            sscanf(strtemp, "%x", &csreceived);//字符串strtemp转换为16进制数
                            //检查校验是否正确
                            if (cscomputed != csreceived)//校验和不匹配
                            {
                                memset(OneFrame, 0, sizeof(OneFrame));
                                GTIMU_STATE_PARSER = 0;
                                ROS_ERROR_STREAM("IMU ERROR IN CASE4.");
                            } else//校验和匹配
                            {
                                GTIMU_STATE_PARSER = 5;
                            }
                        }//校验和字节是hex
                        break;
                    case 5: //等待结束标志<CR>=0x0d
                        OneFrame[CntByte] = rbuf[i];
                        CntByte++;//指向下一个空位
                        if (rbuf[i] == '\r') {
                            //printf("End flag <CR>=0x0%x OK!\n",rbuf[i]);
                            GTIMU_STATE_PARSER = 6;
                        } else {
                            printf("ERROR IN CASE 6.");
                            memset(OneFrame, 0, sizeof(OneFrame));
                            GTIMU_STATE_PARSER = 0;
                            ROS_ERROR_STREAM("DATA ERROR IN CASE6.");
                            //printf("End flag <CR> error!\n");
                        }
                        break;
                    case 6: //等待结束标志<LF>=0x0a
                        OneFrame[CntByte] = rbuf[i];
                        if (rbuf[i] == '\n') {
                            //printf("A good frame!\n");
                            ROS_INFO_STREAM("IMU DATA OK");
                        } else {
                            //printf("End flag <LF> error!\n");
                            ROS_ERROR_STREAM("DATA ERROR IN CASE6.");
                        }
                        memset(OneFrame, 0, sizeof(OneFrame));
                        GTIMU_STATE_PARSER = 0;
                        break; //结束了数据解析
                    default:
                        memset(OneFrame, 0, sizeof(OneFrame));
                        GTIMU_STATE_PARSER = 0;
                        break;
                }//switch
            }// protocol GTIMU

        }// for i in numgeted
    }// run Algorithm


}// ns_starneto_mems
