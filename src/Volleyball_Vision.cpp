#include"Image.cpp"
using namespace std;
cv::Mat volleyball;
std::timed_mutex mut;
bool start=false;
bool end_thred=true;
void thread_image(std::shared_ptr<Image> image1){
    int cont=0;
  while(end_thred){
        //std::cout<<"进去"<<std::endl;
        if(start){
        std::unique_lock<std::timed_mutex> lg(mut,std::defer_lock);
                    cv::Mat ball;
        if(lg.try_lock_for(std::chrono::seconds(1)))
        {
            ball=volleyball.clone();
            start=false;
        }
        image1->Image_processing(volleyball);
            float send_x=(float)image1->x;
            float send_y=(float)image1->y;
            //image1->Float_to_Byte(send_x, send_y, image1->deep,image1->ratio, sp.send_date);//转float
            //std::string message(reinterpret_cast<char *>(sp.send_date), sizeof(sp.send_date));
            //sp.send_message(message);
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
    //std::thread thread=std::thread(thread_image,image);
    std::chrono::steady_clock::time_point prev_time = std::chrono::steady_clock::now();  
    boost::asio::io_service io; // 调用IO口
    SerialPort sp(io, "/devACM0"); 
    int frame_count = 0;  
    double fps = 0.0;
    long preTime;
    while (true)
    {
        preTime = chrono::duration_cast<chrono::milliseconds>(
            chrono::system_clock::now().time_since_epoch())
            .count();
        // cv::Mat l;
        // std::unique_lock<std::timed_mutex> lg(mut,std::defer_lock);
        // if(lg.try_lock_for(std::chrono::seconds(1)))
        // {
             volleyball=image->Image_Get();
        //     start=true;
        //     l=volleyball.clone(); 
        // }
        image->Image_processing(volleyball);
        float send_x=(float)image->x;
        float send_y=(float)image->y;
        image->Float_to_Byte(send_x, send_y, image->deep,image->ratio, sp.send_date);//转float
        std::string message(reinterpret_cast<char *>(sp.send_date), sizeof(sp.send_date));
        sp.send_message(message);
        cv::imshow("i",volleyball);
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
    //thread.join();  
    return 0;
}



    