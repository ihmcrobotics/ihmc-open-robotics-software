#include "visual_odometry_external.h"
#include "iostream"

VisualOdometryExternal::VisualOdometryExternal()
{
   std::cout << "VisualOdometryExternal Created" << std::endl;

   _visualOdometry = new VisualOdometry(_appState);
}

void VisualOdometryExternal::printMat(float* buffer, int height, int width)
{
    std::cout << "Height: " << height << " Width: " << width << std::endl;

    for(int i = 0; i<height; i++)
    {
        for(int j = 0; j<width; j++)
        {
            printf("%.2lf\t", buffer[i*width + j]);
        }
        printf("\n");
    }
}

void VisualOdometryExternal::printMat(uint8_t* buffer, int height, int width)
{
    std::cout << "[IntBuffer] Height: " << height << " Width: " << width << std::endl;

    

    for(int i = 0; i<height; i++)
    {
        for(int j = 0; j<width; j++)
        {
            printf("%d\t", buffer[i*width + j]);
        }
        printf("\n");
    }
}

void VisualOdometryExternal::displayMat(uint8_t* buffer, int height, int width, int delayMilliSeconds)
{
    cv::Mat mat(height, width, CV_8UC1, buffer);
    cv::Mat display;
    cv::cvtColor(mat, display, cv::COLOR_GRAY2BGR);

    cv::imshow("Image", display);
    cv::waitKey(delayMilliSeconds);
}

// void VisualOdometryExternal::updateMonocular(uint8_t* buffer, int height, int width)
// {
//     cv::Mat mat(height, width, CV_8UC1, buffer);
//     cv::Mat final;
//     cv::cvtColor(mat, final, cv::COLOR_GRAY2BGR);

//     _visualOdometry->UpdateMonocular(final);
// }

void VisualOdometryExternal::updateStereo(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width)
{
    cv::Mat matLeft(height, width, CV_8UC1, bufferLeft);
    cv::Mat matRight(height, width, CV_8UC1, bufferRight);

    // cv::cvtColor(matLeft, matLeft, cv::COLOR_GRAY2BGR);
    // cv::cvtColor(matRight, matRight, cv::COLOR_GRAY2BGR);

    _visualOdometry->UpdateStereo(matLeft, matRight);

    // Extract keypoints and descriptors from left image

    // Extract keypoints and descriptors from right image

    // Match between left and right feature points

    // Match between previous and current left feature points

    // Match between previous and current right feature points

    // Triangulate points from stereo matches

    // Insert point landmarks into map at 

    // Map is a graph of keyframes and point landmarks. Indexing into list of keyframes and list of point landmarks

    // Estimate pose 2D-2D between previous keyframe(left) and current left frame

    // Find common keypoints among last keyframe map and current stereo common 2d measurements

}

void VisualOdometryExternal::getExternalKeyframe(float* odometry, uint32_t* id)
{
    *(id) = _visualOdometry->GetLastKeyframe().id;
    std::copy(_visualOdometry->GetLastKeyframe().pose.data(),
                _visualOdometry->GetLastKeyframe().pose.data() + 16,
                odometry);
}

uint32_t VisualOdometryExternal::getExternalLandmarks(float* landmarksToPack, uint32_t* idsToPack, uint32_t maxSize)
{
    auto landmarksVec = _visualOdometry->GetLandmarkVec();
    for(uint32_t i = 0; i<landmarksVec.size(); i++)
    {
        if(i < maxSize)
        {
            idsToPack[i] = landmarksVec[i].GetLandmarkID();

            landmarksToPack[i*5] = landmarksVec[i].GetMeasurement2D().x();
            landmarksToPack[i*5 + 1] = landmarksVec[i].GetMeasurement2D().y();
            landmarksToPack[i*5 + 2] = landmarksVec[i].GetPoint3D().x();
            landmarksToPack[i*5 + 3] = landmarksVec[i].GetPoint3D().y();
            landmarksToPack[i*5 + 4] = landmarksVec[i].GetPoint3D().z();
        }
    }
    return (landmarksVec.size() < maxSize ? landmarksVec.size() : maxSize);
}

// void VisualOdometryExternal::testStereoFeatureExtraction(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width)
// {

// }

// void VisualOdometryExternal::testStereoFeatureMatching(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width)
// {

// }

// void VisualOdometryExternal::testStereoDisparityGeneration(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width)
// {

// }

// void VisualOdometryExternal::testStereoTriangulation(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width)
// {

// }

// void VisualOdometryExternal::testMotionEstimation(uint8_t* bufferLeft, uint8_t* bufferRight, int height, int width)
// {

// }

