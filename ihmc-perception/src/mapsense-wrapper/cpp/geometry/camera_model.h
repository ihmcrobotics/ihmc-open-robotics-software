#pragma once

#include "Eigen/Core"
#include "opencv4/opencv2/opencv.hpp"
#include "boost/format.hpp"

class CameraModel
{
   public:
      CameraModel() {};
      CameraModel(float fx, float fy, float cx, float cy);

      void SetParams(float fx, float fy, float cx, float cy)
      {
         _fx = fx;
         _fy = fy;
         _cx = cx;
         _cy = cy;
      };

      float _fx, _fy, _cx, _cy;

   private:

      Eigen::Matrix3f _cameraMatrix;
      Eigen::Matrix4f _transform;

};
