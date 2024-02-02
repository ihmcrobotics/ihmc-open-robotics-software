#pragma once

#include "Eigen/Core"
#include "vector"

class PointLandmark
{

   public:
      PointLandmark(Eigen::Vector3f point3D) : _point3D(point3D){};

      PointLandmark(Eigen::Vector3f point3D, Eigen::Vector2f measurement2D, int id) : _point3D(point3D), _measurement2D(measurement2D), _id(id){};

      PointLandmark(Eigen::Vector3f point3D, Eigen::Vector2f measurement2D, cv::Mat descriptor, int id)
         : _descriptor(descriptor), _point3D(point3D), _measurement2D(measurement2D), _id(id){};

      const Eigen::Vector3f& GetPoint3D() const {return _point3D;}

      const Eigen::Vector2f& GetMeasurement2D() const {return _measurement2D;}

      void SetPoint3D(float x, float y, float z) {_point3D.x() = x; _point3D.y() = y; _point3D.z() = z;}

      void SetMeasurement2D(float x, float y) {_measurement2D.x() = x; _measurement2D.y() = y;}

      void SetLandmarkID(uint32_t id) {_id = id;}

      uint32_t GetLandmarkID() const {return _id;};

   public:
      cv::Mat _descriptor;
      Eigen::Vector3f _point3D;
      Eigen::Vector2f _measurement2D;

      int _id = -1;

};