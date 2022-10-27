//
// Created by quantum on 12/16/21.
//

#include "camera_model.h"

CameraModel::CameraModel(float fx, float fy, float cx, float cy)  : _fx(fx), _fy(fy), _cx(cx), _cy(cy)
{
   _cameraMatrix = Eigen::Matrix3f::Identity();
   _cameraMatrix(0,0) = _fx;
   _cameraMatrix(1,1) = _fy;
   _cameraMatrix(0,2) = _cx;
   _cameraMatrix(1,2) = _cy;

//   cv::Mat camMat(_cameraMatrix.rows(), _cameraMatrix.cols(), CV_32FC1, _cameraMatrix.data());
//   _cvCameraMatrix = camMat;
}




