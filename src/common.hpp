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
class LowPassFilter {
public:
    LowPassFilter(double dt, double RC) : dt(dt), RC(RC), prev_y(0), first_run(true) {}

    double filter(double x) {
        if (first_run) {
            first_run = false;
            prev_y = x;
        }
        double alpha = dt / (RC + dt);
        double y = alpha * x + (1 - alpha) * prev_y;
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
    uint8_t send_date[12];

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
 * @brief 限制数据范围
 *
 * @param date
 * @param min
 * @param max
 * @return float
 */
// float limit_data(float date , float min, float max)
// {
//     if (date < min)
//     {
//         date = min;
//     }
//     if (date > max)
//     {
//         date = max;
//     }
//     return date;
// }
/**
 * @brief 将浮点数f转化为4个字节数据存放在byte[4]中
 *
 */
typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;
/**
 * @brief 将浮点数f转化为4个字节数据存放在byte[4]中
 *
 * @param a
 * @param b
 * @param c
 * @param byte
 */
// void Float_to_Byte(float a, float b, float c, unsigned char byte[])
// {
//     FloatLongType fl, f2 ,f3;   
//     fl.fdata = a;
//     f2.fdata = b;
//     f3.fdata = c;
//     byte[0] = (unsigned char)fl.ldata;
//     byte[1] = (unsigned char)(fl.ldata >> 8);
//     byte[2] = (unsigned char)(fl.ldata >> 16);
//     byte[3] = (unsigned char)(fl.ldata >> 24);
//     byte[4] = (unsigned char)f2.ldata;
//     byte[5] = (unsigned char)(f2.ldata >> 8);
//     byte[6] = (unsigned char)(f2.ldata >> 16);
//     byte[7] = (unsigned char)(f2.ldata >> 24);
//     byte[8] = (unsigned char)f3.ldata;
//     byte[9] = (unsigned char)(f3.ldata >> 8);
//     byte[10] = (unsigned char)(f3.ldata >> 16);
//     byte[11] = (unsigned char)(f3.ldata >> 24);
// }
#endif