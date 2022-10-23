#pragma once

#include <Eigen/Dense>
#include <memory>
#include <unordered_map>

#include "region_ring.h"
#include "rigid_body_transform.h"



class PlanarRegion
{
   public:
      PlanarRegion() = default;

      PlanarRegion(int id);

      std::vector<Eigen::Vector3f> boundaryVertices;

      void SubSampleBoundary(int skip);

      void ComputeSegmentIndices(float distThreshold);

      void ComputeBoundaryVerticesPlanar();

      void ComputeBoundaryVertices3D(std::vector<Eigen::Vector2f> points2D);

      void RetainLinearApproximation();

      void RetainConvexHull();

      int GetNumOfMeasurements() const;

      void SetNumOfMeasurements(int numOfMeasurements);

      int GetPoseId() const;

      void setPoseId(int poseId);

      Eigen::Vector3f GetPCANormal();

      Eigen::Vector3f getMeanCenter();

      Eigen::Vector3f GetMeanNormal();

      void AddPatch(Eigen::Vector3f normal, Eigen::Vector3f center);

      void insertBoundaryVertex(Eigen::Vector3f vertex);

      void insertLeafPatch(Eigen::Vector2i pos);

      void GetClockWise2D(std::vector<Eigen::Vector2f>& points);

      void SortOrderClockwise();

      std::vector<Eigen::Vector3f> getBoundaryVertices();

      int GetNumOfBoundaryVertices();

      Eigen::Vector3f GetNormal();

      Eigen::Vector3f GetCenter();

      std::vector<Eigen::Vector3f> getVertices();

      std::vector<Eigen::Vector2i> getLeafPatches();

      int getNumPatches();

      int getId();

      void setId(int id);

      void SetNormal(const Eigen::Vector3f& normal);

      void SetCenter(const Eigen::Vector3f& center);

      void WriteToFile(std::ofstream& file);

      std::vector<std::shared_ptr<RegionRing>> rings;

      void transform(RigidBodyTransform transform);

      void transform(Eigen::Vector3d translation, Eigen::Matrix3d rotation);

      void TransformAndFill(std::shared_ptr<PlanarRegion>& planarRegionToPack, RigidBodyTransform transform);

      const std::string& toString();

      void ProjectToPlane(const Eigen::Vector4f& plane);

      void SetToUnitSquare();

      static void PrintRegionList(const std::vector<std::shared_ptr<PlanarRegion>>& regionList, const std::string& name);

      static void SetZeroId( std::vector<std::shared_ptr<PlanarRegion>>& regionList);

      const std::vector<Eigen::Vector2f>& GetPlanarPatchCentroids() const {return boundaryVerticesPlanar;}

      void SetPlanarPatchCentroids(std::vector<Eigen::Vector2f> points) { boundaryVerticesPlanar = points;}

      void SetSegmentIndices(std::vector<int> indices) { _segmentIndices = indices;}

      const std::vector<int>& GetSegmentIndices() const {return _segmentIndices;}

      void CompressRegionSegmentsLinear(float compressDistThreshold, float compressCosineThreshold);

      Eigen::Vector4f GetPlane() {
         Eigen::Vector4f plane;
         plane << GetNormal(), -GetNormal().dot(GetCenter());
         return plane;
      }

   private:
      Eigen::Vector3f normal;
      Eigen::Vector3f center;
      std::vector<Eigen::Vector3f> patchCentroids;
      std::vector<Eigen::Vector2f> boundaryVerticesPlanar;
      std::vector<Eigen::Vector2i> leafPatches;
      std::vector<int> _segmentIndices;

      RigidBodyTransform transformToWorldFrame;
      bool normalCalculated = false;
      bool centroidCalculated = false;
      int numPatches;
      int id;
      int poseId = 0;
      int numOfMeasurements = 1;
};

struct PlanarRegionSet
{
   private:
      std::unordered_map<int, int> _indexMap;
      std::vector<std::shared_ptr<PlanarRegion>> _regions;
      int poseId = 0;
   public:
      int GetID() const { return poseId;}
      void SetID(int id) { poseId = id;}

      void InsertRegion(std::shared_ptr<PlanarRegion>&& region, int id) {
//         if (_indexMap.find(id) == _indexMap.end())
         {
            _regions.emplace_back(region);
            _indexMap[region->getId()] = _regions.size() - 1;
         }
      } // Insert if not present.

      std::unordered_map<int, int>& GetIndices() { return _indexMap;}

      std::vector<std::shared_ptr<PlanarRegion>>& GetRegions() {return _regions;}

      bool Exists(int key) {return _indexMap.find(key) != _indexMap.end(); }

      void Print()
      {
         printf("\n\nPlanarRegionSet: %d\n", _regions.size());
         for(auto region : _regions)
            printf("%s\n", region->toString().c_str());
      }

};

