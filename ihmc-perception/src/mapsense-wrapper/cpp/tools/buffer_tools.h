#pragma once

#include "opencv4/opencv2/core/core.hpp"

class BufferTools
{
    public:
        static void PrintMatR8(cv::Mat& mat, int value, bool invert, bool constant, int rowLimit, int colLimit);
        static void PrintMatR16(cv::Mat& mat, int value, bool invert, int rowLimit, int colLimit, bool linear);
}