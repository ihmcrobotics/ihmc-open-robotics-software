#pragma once

#include "Eigen/Core"
#include "vector"

class PointLandmark
{

   public:
      PointLandmark(Eigen::Vector3f point3D) : _point3D(point3D){};

      const Eigen::Vector3f& GetPoint3D() const {return _point3D;}
      const std::vector<Eigen::Vector2f>& GetMeasurements2D() const {return _measurements2D;}
      const std::vector<int>& GetCameraIDs() const {return _cameraIDs;}

      void AddMeasurement2D(Eigen::Vector2f measurement, int index, int cameraID)
      {
         _measurements2D.emplace_back(measurement);
         _cameraIDs.emplace_back(cameraID);
         _index.emplace_back(index);
      }
      void AddCameraID(int id) {_cameraIDs.emplace_back(id); }
      void SetPoint3D(float x, float y, float z) {_point3D.x() = x; _point3D.y() = y; _point3D.z() = z;}

   public:
      Eigen::Vector3f _point3D;
      std::vector<Eigen::Vector2f> _measurements2D;
      std::vector<int> _cameraIDs;
      std::vector<int> _index;

};
