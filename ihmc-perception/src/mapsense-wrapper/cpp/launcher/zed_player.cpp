#include "zed_open_driver.h"


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main()
{
   const bool DISPLAY = true;


   sl_oc::video::VideoParams params;
   params.res = sl_oc::video::RESOLUTION::HD720;
   params.fps = sl_oc::video::FPS::FPS_60;

   sl_oc::video::VideoCapture cap(params);

   
   if (!cap.initializeVideo())
   {
      std::cerr << "Cannot open camera video capture" << std::endl;
      std::cerr << "See verbosity level for more details." << std::endl;
      return EXIT_FAILURE;
   }
   std::cout << "Connected to camera sn: " << cap.getSerialNumber() << std::endl;

   int count = 0;
   while (true)
   {
      const sl_oc::video::Frame frame = cap.getLastFrame();
      if (frame.data != nullptr)
      {
         cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
         cv::Mat frameBGR;
         cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

         cv::Mat left = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
         cv::Mat right = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

         if (DISPLAY)
         {
            cv::imshow("Left RGB", left);
            cv::imshow("Right RGB", right);

            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') // Quit
               break;
         }
      }

      ++count;
   }

   return EXIT_SUCCESS;
}