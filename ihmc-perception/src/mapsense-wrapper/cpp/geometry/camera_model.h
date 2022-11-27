#pragma once

#include "Eigen/Core"
#include "opencv4/opencv2/opencv.hpp"
#include "boost/format.hpp"

class CameraModel
{
   public:
      CameraModel() {};

      CameraModel(float fx, float fy, float cx, float cy);

      CameraModel(float fx, float fy, float cx, float cy, const Eigen::Matrix4f& transform);

      Eigen::Vector2f Project(const Eigen::Vector4f& point);

      void SetParams(float fx, float fy, float cx, float cy)
      {
         _fx = fx;
         _fy = fy;
         _cx = cx;
         _cy = cy;

         _cameraMatrix(0,0) = fx;
         _cameraMatrix(1,1) = fy;
         _cameraMatrix(0,2) = cx;
         _cameraMatrix(1,2) = fx;
         _cameraMatrix(2,2) = 1;

      };

      void SetTransform(const Eigen::Matrix4f& transform)
      {
         _transform = transform.block<3,4>(0,0);
      }
      
      bool Project(const Eigen::Vector3f& point3d, Eigen::Vector3f& camPoint, Eigen::Vector2f& point2d);


      float _fx, _fy, _cx, _cy;

   private:

      Eigen::Matrix3f _cameraMatrix;
      Eigen::Matrix<float, 3,4> _transform;

};
