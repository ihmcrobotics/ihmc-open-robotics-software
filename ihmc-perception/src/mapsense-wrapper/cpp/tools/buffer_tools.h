#pragma once

#include "opencv4/opencv2/core/core.hpp"

class BufferTools
{
    public:
        static void PrintMatR8(cv::Mat& mat, int value = -1, bool invert = false, bool constant = false, int rowLimit = 0, int colLimit = 0);
        static void PrintMatR16(cv::Mat& mat, int value = -1, bool invert = false, int rowLimit = 0, int colLimit = 0, bool linear = false);
};