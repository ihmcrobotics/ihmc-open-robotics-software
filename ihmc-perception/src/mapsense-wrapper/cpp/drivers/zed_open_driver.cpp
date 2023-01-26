#include "zed_open_driver.h"


ZEDOpenDriver::ZEDOpenDriver() : _cap(_params)
{
    if (!_cap.initializeVideo())
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;
    }
    std::cout << "Connected to camera sn: " << _cap.getSerialNumber() << std::endl;
}

void ZEDOpenDriver::Update(bool display)
{
    _count++;

    const sl_oc::video::Frame frame = _cap.getLastFrame();
    

    if (frame.data != nullptr)
    {
        cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR;
        cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

        cv::Mat left = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
        cv::Mat right = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

        if (display)
        {
            cv::imshow("Left RGB", left);
            cv::imshow("Right RGB", right);

            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') // Quit
                exit(0);
        }
    }
}

