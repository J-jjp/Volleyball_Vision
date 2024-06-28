#include "common.hpp"
#define PI M_PI
using namespace std;
class Image
{
public:
    Image() : align_to_color(RS2_STREAM_COLOR)
    {
        // cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 0);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 0);
        rs2::pipeline_profile profile = pipe.start(cfg); // 请根据实际情况修改串口设备路径
    }
    void Image_processing(cv::Mat &image)
    {

        // 计算总像素数
        int totalPixels = image.total();
        // 计算白色和黑色像素数
        int whitePixels = cv::countNonZero(image);
        GaussianBlur(image, image, cv::Size(9, 9), 2, 2);
        // 计算白色像素占比
        float whiteRatio = (float)whitePixels / (float)totalPixels;
        // std::cout<<"whiteRatio"<<whiteRatio<<std::endl;
        cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        // 遍历轮廓
        for (int i = 0; i < hierarchy.size(); i++)
        {
            if (contours[i].size() < 5)
                continue;
            double area = cv::contourArea(contours[i]);
            if (area < 20)
                continue;
            double arc_length = cv::arcLength(contours[i], true);
            double radius = arc_length / (2 * PI);

            if (!(MinR < radius && radius < MaxR))
            {
                continue;
            }

            cv::RotatedRect rect = cv::fitEllipse(contours[i]);
            float ratio = float(rect.size.width) / float(rect.size.height);

            if (ratio < 1.6 && ratio > 0.7)
            // 因为圆的外接直立矩形肯定近似于一个正方形，因此宽高比接近1.0
            {
                std::cout<<"W:"<<whiteRatio<<std::endl;
                if (whiteRatio > 0.05)
                {
                    int center_x = limit_data(rect.center.x, 1, width-1);
                    int center_y = limit_data(rect.center.y, 1, height-1);
                    x = lpf.filter(center_x);
                    y = lpf.filter(center_y);
                    printf("X: %d\n",x);
                    printf("Y: %d\n", y);
                    ratio=whiteRatio;
                    deep =n[x][y];
                    if(whiteRatio > 0.9) {deep = 0.15;}  //判断球到脸上无法识别的情况
                    printf("D: %f\n",deep);

                }
                // if (has_blue == true && has_yellow == true)
                // {
                // 发送数据
                // sp.ball_tracking_pos.x = limit_data(rect.center.x , 1 , 640 );
                // sp.ball_tracking_pos.y = limit_data(rect.center.y , 0 , 480 );
                // sp.ball_tracking_pos.deep = depth_frame.get_distance(sp.ball_tracking_pos.x , sp.ball_tracking_pos.y );

                // if(whiteRatio > 0.94) {sp.ball_tracking_pos.deep = 0.15;}  //判断球到脸上无法识别的情况
                // std::string message(reinterpret_cast<char *>(sp.send_date), sizeof(sp.send_date));

                // printf("D: %f\n", sp.ball_tracking_pos.deep);
                // sp.send_message(message);
                // std::cout<<"有"<<std::endl;
                //  如果非debug关闭绘图，这是一个及其耗时的操作
                //  cv::ellipse(color_image, rect, cyan_blue, 2);
                //  cv::circle(color_image, rect.center, 2, green, 2, 8, 0);
            }
            else
            {
                // sp.ball_tracking_pos.x = 320;0
                x = 320;
                y = 240;
                if(whiteRatio > 0.94) {deep = 0.15;}
                deep=0.0;
                //     //Float_to_Byte(sp.ball_tracking_pos.x, sp.ball_tracking_pos.y, sp.ball_tracking_pos.deep, sp.send_date);

                //     //std::string message(reinterpret_cast<char *>(sp.send_date), sizeof(sp.send_date));
                //     printf("未识别到球\n");
                //     //sp.send_message(message);
            }
        }
        // 如果非debug关闭imshow和waitKey，这是一个及其耗时的操作
        // printf("whiteRatio: %f\n", whiteRatio);
        // cv::imshow("Color Image", color_image);
        // cv::imshow("Color", binaryImage);
        // cv::waitKey(1);
    }
    cv::Mat Image_Get()
    {
        rs2::frameset frames = pipe.wait_for_frames();
        // auto aligned_frames = align_to_color.process(frames);
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
        // cv::Mat color_image(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        int _width = depth_frame.get_width();
        int _height = depth_frame.get_height();
        const uint16_t *depth_data = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
        int depth_pitch = depth_frame.get_stride_in_bytes() / sizeof(uint16_t);
        cv::Mat de_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        depth_image = de_image.clone();
        cv::Mat blackImage = cv::Mat::zeros(height, width, CV_8UC1);
        for (int x = 0; x < _height; ++x)
        {
            for (int y = 20; y < _width; ++y)
            {
                int index = x * depth_pitch + y;
                uint16_t depth_value = depth_data[index];
                n[x][y]=depth_frame.get_distance(y,x);
                if (depth_value < maxDistance&&depth_value>0)
                {
                    blackImage.at<uchar>(x, y) = 255;
                    
                }
                else
                {
                    blackImage.at<uchar>(x, y) = 0;
                }
            }
        }
        return blackImage;
    }
    int MinR = 20;               // 最小像素半径
    int MaxR = 400;              // 最大像素半径
    float maxDistance = 1000.0f; // 测量的最远距离
    int width = 640;             // 图像的宽度
    int height = 360;            // 图像的高度
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align_to_color;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    LowPassFilter lpf; // 创建滤波器，你可以根据需要调整dt和RC的值
    int x = 0;
    int y = 0;
    float deep = 0;
    uint8_t ball_flag = 0;
    float ratio=0;
    cv::Mat depth_image;
    float n[360][640];
    /**
     * @brief 限制数据范围
     *
     * @param date
     * @param min
     * @param max
     * @return float
     */
    int limit_data(int date, int min, int max)
    {
        if (date < min)
        {
            date = min;
        }
        if (date > max)
        {
            date = max;
        }
        return date;
    }
    /**
     * @brief 将浮点数f转化为4个字节数据存放在byte[4]中
     *
     * @param a
     * @param b
     * @param c
     * @param byte
     */
    void Float_to_Byte(float a, float b, float c, float d,unsigned char byte[])
    {
        FloatLongType fl, f2, f3,f4;
        fl.fdata = a;
        f2.fdata = b;
        f3.fdata = c;
        f4.fdata = d;
        byte[0] = (unsigned char)fl.ldata;
        byte[1] = (unsigned char)(fl.ldata >> 8);
        byte[2] = (unsigned char)(fl.ldata >> 16);
        byte[3] = (unsigned char)(fl.ldata >> 24);
        byte[4] = (unsigned char)f2.ldata;
        byte[5] = (unsigned char)(f2.ldata >> 8);
        byte[6] = (unsigned char)(f2.ldata >> 16);
        byte[7] = (unsigned char)(f2.ldata >> 24);
        byte[8] = (unsigned char)f3.ldata;
        byte[9] = (unsigned char)(f3.ldata >> 8);
        byte[10] = (unsigned char)(f3.ldata >> 16);
        byte[11] = (unsigned char)(f3.ldata >> 24);
        byte[12] = (unsigned char)f4.ldata;
        byte[13] = (unsigned char)(f4.ldata >> 8);
        byte[14] = (unsigned char)(f4.ldata >> 16);
        byte[15] = (unsigned char)(f4.ldata >> 24);
    }
};
