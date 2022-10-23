#pragma once

#include "Eigen/Dense"

#include "planar_region.h"
#include <stack>
#include <iostream>
#include <string>
#include <fstream>

class GeomTools
{
   public:


      static Eigen::Matrix3f GetRotationFromAngleApproximations(Eigen::Vector3f eulerAngles);

      static Eigen::Vector3f GetProjectedPoint(Eigen::Vector4f plane, Eigen::Vector3f point);

      static void LoadRegions(int frameId, std::vector<std::shared_ptr<PlanarRegion>>& regions, std::string directory, std::vector<std::string> files);

      static void LoadRegions(const std::string& file, std::vector<std::shared_ptr<PlanarRegion>>& regions, bool erase = false);

      static void SaveRegions(std::vector<std::shared_ptr<PlanarRegion>> regions, std::string fileName);

      static void TransformRegions(std::vector<std::shared_ptr<PlanarRegion>>& regions, RigidBodyTransform transform);

      static void TransformRegions(std::vector<std::shared_ptr<PlanarRegion>>& regions, Eigen::Vector3d translation, Eigen::Matrix3d rotation);

      static void LoadPoseStamped(std::ifstream& poseFile, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

      static float GetCosineSimilarity2D(const Eigen::Vector2f& a, const Eigen::Vector2f& b);

      static bool
      CheckPatchConnection(const Eigen::Vector3f& ag, const Eigen::Vector3f& an, const Eigen::Vector3f& bg, const Eigen::Vector3f& bn, float distanceThreshold,
                           float angularThreshold);

      static void
      AppendMeasurementsToFile(const Eigen::Matrix4f odometry, const std::vector<std::pair<int, int>>& matches, const std::string& filename, int prevId,
                               int curId);

};
