#ifndef COMMON_HPP
#define COMMON_HPP

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <termios.h>
#include <boost/asio.hpp>
#define _USE_MATH_DEFINES
/**
 * @brief 低通滤波类
 *
 */
class LowPassFilter
{
public:
    int filter(double x)
    {
        float alpha = 0.8;
        int y = alpha * x + (1 - alpha) * prev_y;
        prev_y = y;
        return y;
    }

private:
    double dt;
    double RC;
    double prev_y;
    bool first_run;
};
/**
 * @brief 串口通信类
 *
 */
class SerialPort
{
public:
    uint8_t send_date[15];

    typedef struct
    {
        float x;
        float y;
        float deep;
        uint8_t ball_flag;
    } Ball_tracking_Pos;

    /// 初始化数据结构体/////
    Ball_tracking_Pos ball_tracking_pos; // 真实球的位置
    // Ball_tracking_Pos ball_tracking_prediction_pos; // 预测球的位置

    SerialPort(boost::asio::io_service &io, const std::string &port)
        : serial(io, port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200)); // 设置波特率
    }

    void send_message(const std::string &message)
    {
        boost::asio::write(serial, boost::asio::buffer(message));
    }

private:
    boost::asio::serial_port serial;
};
/**
 * @brief 将浮点数f转化为4个字节数据存放在byte[4]中
 *
 */
typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

#endif