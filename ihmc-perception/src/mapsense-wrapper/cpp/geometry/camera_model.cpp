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
}

CameraModel::CameraModel(float fx, float fy, float cx, float cy, const Eigen::Matrix4f& transform)  
   : _fx(fx), _fy(fy), _cx(cx), _cy(cy)
{
   _cameraMatrix = Eigen::Matrix3f::Identity();
   _cameraMatrix(0,0) = _fx;
   _cameraMatrix(1,1) = _fy;
   _cameraMatrix(0,2) = _cx;
   _cameraMatrix(1,2) = _cy;

   _transform = transform.block<3,4>(0,0);
}

bool CameraModel::Project(const Eigen::Vector3f& point3d, Eigen::Vector3f& camPoint, Eigen::Vector2f& point2d)
{
   Eigen::Vector4f point;
   point << point3d, 1;

   // Set Camera Frame 3D Point
   camPoint = _transform * point;

   // Return if Camera Frame Point is behind the camera   
   if (camPoint.z() <= 0)
   {
      return false;
   }

   // Set Image Frame 2D Point
   Eigen::Vector3f projection = _cameraMatrix * camPoint;
   point2d.x() = projection.x() / projection.z();
   point2d.y() = projection.y() / projection.z();

   return camPoint.z() > 0;
}



