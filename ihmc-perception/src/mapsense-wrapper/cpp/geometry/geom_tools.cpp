#include "geom_tools.h"

Eigen::Matrix3f GeomTools::GetRotationFromAngleApproximations(Eigen::Vector3f eulerAngles)
{
   float alpha = eulerAngles.x();
   float beta = eulerAngles.y();
   float gamma = eulerAngles.z();
   Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
   rotation(0, 1) = alpha * beta - gamma;
   rotation(0, 2) = alpha * gamma + beta;
   rotation(1, 0) = gamma;
   rotation(1, 1) += alpha * beta * gamma;
   rotation(1, 2) = beta * gamma - alpha;
   rotation(2, 0) = -beta;
   rotation(2, 1) = alpha;
   return rotation;
}

Eigen::Vector3f GeomTools::GetProjectedPoint(Eigen::Vector4f plane, Eigen::Vector3f point)
{
   Eigen::Vector3f normal = plane.block<3, 1>(0, 0).normalized();
   return point - normal * (normal.dot(point) + plane(3) / plane.block<3, 1>(0, 0).norm());
}

float GeomTools::GetCosineSimilarity2D(const Eigen::Vector2f& a, const Eigen::Vector2f& b)
{
   return a.dot(b) / (a.norm() * b.norm());
}



Eigen::Vector3f GetVec3f(std::string csv)
{
   std::vector<std::string> CSVSubStrings;
   std::stringstream csvStream(csv);
   std::string csvStr;
   while (getline(csvStream, csvStr, ','))
   {
      CSVSubStrings.push_back(csvStr);
   }
   return Eigen::Vector3f(stof(CSVSubStrings[0]), stof(CSVSubStrings[1]), stof(CSVSubStrings[2]));
}

void GetNextLineSplit(std::ifstream& regionFile, std::vector<std::string>& subStrings, char delimiter = ',')
{
   subStrings.clear();
   std::string regionText;
   getline(regionFile, regionText);
   std::stringstream ss(regionText);
   std::string str;
   while (getline(ss, str, delimiter))
   {
      subStrings.push_back(str);
   }
}

void GeomTools::SaveRegions(std::vector<std::shared_ptr<PlanarRegion>> regions, std::string fileName)
{
   std::ofstream file;
   file.open(fileName, std::fstream::in | std::fstream::out | std::fstream::app);
   file << "NumRegions:" << regions.size() << std::endl;
   for (std::shared_ptr<PlanarRegion> region: regions)
   {
      region->WriteToFile(file);
   }
   file.close();
   //   std::cout << "Writing Regions to:" << fileName << std::endl;
}

void
GeomTools::AppendMeasurementsToFile(const Eigen::Matrix4f odometry, const std::vector<std::pair<int, int>>& matches, const std::string& filename, int prevId,
                                    int curId)
{
   std::ofstream file;
   file.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
   file << "NumMatches:" << matches.size() << std::endl;
   file << "IDs:" << prevId << "," << curId << std::endl;
   file << "Pose:" << odometry << std::endl;
   for (auto match: matches)
   {
      file << "Match:" << match.first << "," << match.second << std::endl;
   }
   file.close();
}

void GeomTools::LoadRegions(int frameId, std::vector<std::shared_ptr<PlanarRegion>>& regions, std::string directory, std::vector<std::string> files)
{
   /* Generate planar region objects from the sorted list of files. */
   //   if(regions.size() > 0) regions.clear();

   std::string filePath = directory + files[frameId];
   LoadRegions(filePath, regions);
}

void GeomTools::LoadRegions(const std::string& file, std::vector<std::shared_ptr<PlanarRegion>>& regions, bool erase)
{
   if(erase) regions.clear();
   std::ifstream regionFile(file);
   std::cout << "Loading Regions From: " << file << std::endl;
   std::vector<std::string> subStrings;
   GetNextLineSplit(regionFile, subStrings, ':'); // Get number of regions
   int numRegions = stoi(subStrings[1]);
   for (int r = 0; r < numRegions; r++) // For each region
   {
      std::shared_ptr<PlanarRegion> region = std::make_shared<PlanarRegion>(0);
      GetNextLineSplit(regionFile, subStrings, ':'); // Get regionId
      region->setId(-1);
      //      region->setId(stoi(subStrings[1]));
      GetNextLineSplit(regionFile, subStrings, ':'); // Get regionCenter
      region->SetCenter(GetVec3f(subStrings[1]));
      GetNextLineSplit(regionFile, subStrings, ':'); // Get regionNormal
      region->SetNormal(GetVec3f(subStrings[1]));
      GetNextLineSplit(regionFile, subStrings, ':'); // Get numBoundaryVertices
      int length = stoi(subStrings[1]);
      for (int i = 0; i < length; i++)
      {
         //         std::cout << i << " : ";
         GetNextLineSplit(regionFile, subStrings, ',');
         Eigen::Vector3f point = Eigen::Vector3f(stof(subStrings[0]), stof(subStrings[1]), stof(subStrings[2]));
         //         std::cout << point << std::endl;
         region->insertBoundaryVertex(point);
      }
      //      GeomTools::CompressRegionSegmentsLinear(region);
      regions.emplace_back(std::move(region));
   }
}

void GeomTools::LoadPoseStamped(std::ifstream& poseFile, Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
{
   std::vector<std::string> subStrings;
   GetNextLineSplit(poseFile, subStrings);

   position.x() = stof(subStrings[1]);
   position.y() = stof(subStrings[2]);
   position.z() = stof(subStrings[3]);

   orientation.x() = stof(subStrings[4]);
   orientation.y() = stof(subStrings[5]);
   orientation.z() = stof(subStrings[6]);
   orientation.w() = stof(subStrings[7]);

   std::cout << "Loading Pose: " << subStrings[0] << std::endl;
   std::cout << position << std::endl;
   std::cout << orientation.matrix() << std::endl;
}

void GeomTools::TransformRegions(std::vector<std::shared_ptr<PlanarRegion>>& regions, RigidBodyTransform transform)
{
   for (int i = 0; i < regions.size(); i++)
   {
      regions[i]->transform(transform);
   }
}

void GeomTools::TransformRegions(std::vector<std::shared_ptr<PlanarRegion>>& regions, Eigen::Vector3d translation, Eigen::Matrix3d rotation)
{
   for (int i = 0; i < regions.size(); i++)
   {
      regions[i]->transform(translation, rotation);
   }
}

bool GeomTools::CheckPatchConnection(const Eigen::Vector3f& ag, const Eigen::Vector3f& an, const Eigen::Vector3f& bg, const Eigen::Vector3f& bn,
                                     float distanceThreshold, float angularThreshold)
{
   Eigen::Vector3f vec = ag - bg;
   float dist = vec.norm();
   float sim = fabs(an.dot(bn));
   float perpDist = fabs((ag - bg).dot(bn)) + fabs((bg - ag).dot(an));
   if (perpDist < distanceThreshold * dist * 40 && sim > angularThreshold)
   {
      return true;
   } else
   {
      return false;
   }
}





