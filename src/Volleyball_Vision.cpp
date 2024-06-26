#include"Image.cpp"
using namespace std;
cv::Mat volleyball;
std::mutex mut;
bool start=false;
bool end_thred=true;
void thread_image(std::shared_ptr<Image> image1){
  while(end_thred){
        //std::cout<<"进去"<<std::endl;
        if(start){
            mut.lock();
            cv::Mat ball=volleyball.clone();
            start=false;
            mut.unlock();
            image1->Image_processing(ball);
            //std::cout<<"进去2"<<std::endl;
        }
        else{
            cv::waitKey(1);
        }
    }
}
int main()
{
    std::shared_ptr<Image> image = std::make_shared<Image>(); 
    LowPassFilter lpf(0.1, 1.0);  // 创建滤波器，你可以根据需要调整dt和RC的值

    //boost::asio::io_service io; // 调用IO口
    // SerialPort sp(io, "/dev/ttyS0"); // 测试图像用
    //SerialPort sp(io, "/dev/ttyACM0"); // 请根据实际情况修改串口设备路径
    std::thread thread=std::thread(thread_image,image);
    std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();  
    int frame_count = 0;  
    double fps = 0.0;
    long preTime;
    while (true)
    {
        preTime = chrono::duration_cast<chrono::milliseconds>(
            chrono::system_clock::now().time_since_epoch())
            .count();
        volleyball=image->Image_Get();
        mut.lock();
        start=true;
        mut.unlock();
        //cv::imshow("i",volleyball);
        //cv::waitKey(11);
        if(1){
            auto startTime = chrono::duration_cast<chrono::milliseconds>(
                            chrono::system_clock::now().time_since_epoch())
                            .count();
            std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();  
            std::chrono::duration<double> elapsed_seconds = current_time - prev_time;  
            if (elapsed_seconds.count() >= 1)  
            {  
                fps = frame_count / elapsed_seconds.count();
                printf(">> FrameTime: %ldms | %.2ffps \t", startTime - preTime,1000.0 / (startTime - preTime));
                std::cout << "FPS: " << fps << std::endl;  
                // 重置时间点和帧数  
                prev_time = current_time;  
                frame_count = 0;  
            }  
            frame_count++;
        }
    }
    end_thred=false;
    thread.join();  
    return 0;
}



    