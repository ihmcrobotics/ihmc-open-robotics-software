#include "map_frame_processor.h"

#include "tbb/tbb.h"
#include "tbb/parallel_for.h"
#include "tbb/blocked_range.h"

MapFrameProcessor::MapFrameProcessor(ApplicationState& app) : _app(app) {}

void MapFrameProcessor::Init(ApplicationState& app)
{
   // MAPSENSE_PROFILE_FUNCTION();
   // MS_INFO("Initializing MapFrameProcessor");


   if(app.REGION_MODE == 0)
   {
      // MS_INFO("Init MapFrameProcessor: Depth Mode");
      SUB_H = app.SUB_H;
      SUB_W = app.SUB_W;
      INPUT_HEIGHT = app.DEPTH_INPUT_HEIGHT;
      INPUT_WIDTH = app.DEPTH_INPUT_WIDTH;
      PATCH_HEIGHT = app.DEPTH_PATCH_HEIGHT;
      PATCH_WIDTH = app.DEPTH_PATCH_WIDTH;
   }
   else
   {
      // MS_INFO("Init MapFrameProcessor: Hash Mode");
      SUB_H = app.HASH_SUB_H;
      SUB_W = app.HASH_SUB_W;
      INPUT_HEIGHT = app.HASH_INPUT_HEIGHT;
      INPUT_WIDTH = app.HASH_INPUT_WIDTH;
      PATCH_HEIGHT = app.HASH_PATCH_HEIGHT;
      PATCH_WIDTH = app.HASH_PATCH_WIDTH;
   }

   _debugMat = cv::Mat(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3);
   _visited = Eigen::MatrixXi(SUB_H, SUB_W).setZero();
   _boundary = Eigen::MatrixXi(SUB_H, SUB_W).setZero();
   _region = Eigen::MatrixXi(SUB_H, SUB_W).setZero();

   // MS_DEBUG("MapFrameProcessor Initialized.");
}

void MapFrameProcessor::GenerateSegmentation(MapFrame frame, std::vector<std::shared_ptr<PlanarRegion>>& planarRegionList)
{
   // MAPSENSE_PROFILE_FUNCTION();
   // MS_DEBUG("GenerateSegmentation: SW:{}, SH:{}, IH:{}, IW:{}, PH:{}, PW:{}", SUB_W, SUB_H, INPUT_HEIGHT, INPUT_WIDTH, PATCH_HEIGHT, PATCH_WIDTH);

   _frame = frame;

   /* For initial development only. Delete all old previous regions before inserting new ones. Old and new regions should be fused instead. */
   uint8_t components = 0;
   {
      // MAPSENSE_PROFILE_SCOPE("GenerateSegmentation::DFS");
      planarRegionList.clear();
      _visited.setZero();
      _region.setZero();
      _boundary.setZero();
      _debugMat = cv::Scalar(0);
      for (uint16_t i = 0; i < SUB_H; i++)
      {
         for (uint16_t j = 0; j < SUB_W; j++)
         {
            uint8_t patch = frame.patchData.at<uint8_t>(i, j);
            if (!_visited(i, j) && patch == 255)
            {
               int num = 0;
               std::shared_ptr <PlanarRegion> planarRegion = std::make_shared<PlanarRegion>(components);
               DFS(i, j, components, num, _debugMat, planarRegion);
               if (num > _app.REGION_MIN_PATCHES && num - planarRegion->GetNumOfBoundaryVertices() > _app.REGION_BOUNDARY_DIFF)
               {
                  planarRegionList.emplace_back(planarRegion);
                  components++;
               }
            }
         }
      }
   }

   // MS_DEBUG("DFS Generated {} Regions\n", components);

   /* Extract Region Boundary Indices. */
   _visited.setZero();

   FindBoundaryAndHoles(planarRegionList);

   /* Grow Region Boundary. */
   GrowRegionBoundary(planarRegionList);
   // MS_DEBUG("Regions Grown Manually");

}

void MapFrameProcessor::FindBoundaryAndHoles(std::vector<std::shared_ptr<PlanarRegion>>& planarRegionList)
{
   // MAPSENSE_PROFILE_FUNCTION();
   tbb::parallel_for(tbb::blocked_range<int>(0, planarRegionList.size()), [&](const tbb::blocked_range<int>& r)
   {
      for (int i = r.begin(); i < r.end(); i++)
      {
         int components = 0;
         std::vector<Eigen::Vector2i> leafPatches = planarRegionList[i]->getLeafPatches();
         for (int j = 0; j < leafPatches.size(); j++)
         {
            int num = 0;
            std::shared_ptr<RegionRing> regionRing = std::make_shared<RegionRing>(components);
            BoundaryDFS(leafPatches[j].x(), leafPatches[j].y(), planarRegionList[i]->getId(), components, num, _debugMat,
                        regionRing);
            if (num > 3)
            {
               planarRegionList[i]->rings.emplace_back(regionRing);
               components++;
            }
         }
         auto comp = [](const std::shared_ptr<RegionRing> a, const std::shared_ptr<RegionRing> b)
         {
            return a->getNumOfVertices() > b->getNumOfVertices();
         };
         sort(planarRegionList[i]->rings.begin(), planarRegionList[i]->rings.end(), comp);
         //      std::vector<Eigen::Vector2i> ring = planarRegionList[i]->rings[0]->boundaryIndices;
         //      for (int j = 0; j < ring.size(); j++)
         //      {
         //         Vec6f patch = _frame.getRegionOutput().at<Vec6f>(ring[j].x(), ring[j].y());
         //         planarRegionList[i]->insertBoundaryVertex(Eigen::Vector3f(patch[3], patch[4], patch[5]));
         //      }
      }
   });

//   for (int i = 0; i < planarRegionList.size(); i++)
//   {
//      int components = 0;
//      std::vector<Eigen::Vector2i> leafPatches = planarRegionList[i]->getLeafPatches();
//      for (int j = 0; j < leafPatches.size(); j++)
//      {
//         int num = 0;
//         std::shared_ptr<RegionRing> regionRing = std::make_shared<RegionRing>(components);
//         BoundaryDFS(leafPatches[j].x(), leafPatches[j].y(), planarRegionList[i]->getId(), components, num, debug, regionRing);
//         if (num > 3)
//         {
//            planarRegionList[i]->rings.emplace_back(regionRing);
//            components++;
//         }
//      }
//      auto comp = [](const std::shared_ptr<RegionRing> a, const std::shared_ptr<RegionRing> b)
//      {
//         return a->getNumOfVertices() > b->getNumOfVertices();
//      };
//      sort(planarRegionList[i]->rings.begin(), planarRegionList[i]->rings.end(), comp);
////      std::vector<Eigen::Vector2i> ring = planarRegionList[i]->rings[0]->boundaryIndices;
////      for (int j = 0; j < ring.size(); j++)
////      {
////         Vec6f patch = _frame.getRegionOutput().at<Vec6f>(ring[j].x(), ring[j].y());
////         planarRegionList[i]->insertBoundaryVertex(Eigen::Vector3f(patch[3], patch[4], patch[5]));
////      }
//   }
}



void MapFrameProcessor::GrowRegionBoundary(std::vector<std::shared_ptr<PlanarRegion>>& regions)
{
   // MAPSENSE_PROFILE_FUNCTION();
   tbb::parallel_for(tbb::blocked_range<int>(0, regions.size()), [&](const tbb::blocked_range<int>& r){
      for(int i = r.begin(); i<r.end(); i++)
      {
         if(regions[i]->rings.size() <= 0)
         {
            // MS_DEBUG("Region Boundary Size is Zero!");
//            MS_DEBUG(regions[i]->rings.size() == 0, "Region Boundary Size is Zero!");
         }

         Eigen::Vector3f center = regions[i]->GetCenter();
         std::vector<Eigen::Vector2i> ring = regions[i]->rings[0]->boundaryIndices;
         for (int j = 0; j < ring.size(); j++)
         {
            cv::Vec6f patch = _frame.getRegionOutput().at<cv::Vec6f>(ring[j].x(), ring[j].y());
            Eigen::Vector3f vertex(patch[3], patch[4], patch[5]);
            regions[i]->insertBoundaryVertex(vertex + (vertex - center).normalized() * _app.REGION_GROWTH_FACTOR);
         }
      }
   });

   //   for(int i = 0; i<regions.size(); i++)
   //   {
   //      if(regions[i]->rings.size() <= 0)
   //      {
   //         MS_DEBUG("Region Boundary Size is Zero!");
   //         MS_DEBUG(regions[i]->rings.size() == 0, "Region Boundary Size is Zero!");
   //      }
   //
   //      Eigen::Vector3f center = regions[i]->GetCenter();
   //      std::vector<Eigen::Vector2i> ring = regions[i]->rings[0]->boundaryIndices;
   //      for (int j = 0; j < ring.size(); j++)
   //      {
   //         Vec6f patch = _frame.getRegionOutput().at<Vec6f>(ring[j].x(), ring[j].y());
   //         Eigen::Vector3f vertex(patch[3], patch[4], patch[5]);
   //         regions[i]->insertBoundaryVertex(vertex + (vertex - center).normalized() * _app.REGION_GROWTH_FACTOR);
   //      }
   //   }
}


void MapFrameProcessor::printPatchGraph()
{
   // MS_DEBUG("DEBUGGER:({},{}), AppState:({},{})\n", SUB_H, SUB_W, SUB_H, SUB_W);
   for (int i = 0; i < SUB_H; i++)
   {
      for (int j = 0; j < SUB_W; j++)
      {
         uint8_t current = _frame.patchData.at<uint8_t>(i, j);
         if (current == 255)
         {
            printf("1 ");
         } else
         {
            printf("0 ");
         }
      }
      printf("\n");
   }
}

void MapFrameProcessor::DisplayDebugger(int delay)
{
   imshow("DebugOutput", _debugMat);
   int code = cv::waitKeyEx(delay);
   if (code == 113)
   {
      _app.VISUAL_DEPTH_DEBUG = false;
   }
}


///////////////////////////////////////////////////////////////////
/* ----------------- RECURSIVE FUNCTIONS BELOW THIS POINT -------*/
///////////////////////////////////////////////////////////////////

void MapFrameProcessor::BoundaryDFS(int x, int y, int regionId, int component, int& num, cv::Mat& debug, std::shared_ptr<RegionRing> regionRing)
{
   if (_visited(x, y))
      return;

   num++;
   _visited(x, y) = 1;
   regionRing->insertBoundaryIndex(Eigen::Vector2i(x,y));
   if (_app.SHOW_BOUNDARIES)
      circle(debug, cv::Point((y) * PATCH_HEIGHT, (x) * PATCH_WIDTH), 2,
             cv::Scalar((component + 1) * 130 % 255, (component + 1) * 227 % 255, (component + 1) * 332 % 255), -1);
   for (int i = 0; i < 8; i++)
   {
      if (x + adx[i] < SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < SUB_W - 1 && y + ady[i] > 1)
      {
         if (_boundary(x + adx[i], y + ady[i]) == 1 && regionId == _region(x + adx[i], y + ady[i]))
         {
            BoundaryDFS(x + adx[i], y + ady[i], regionId, component, num, debug, regionRing);
         }
      }
   }
   if (_app.VISUAL_DEPTH_DEBUG)
      DisplayDebugger(_app.VISUAL_DEBUG_DELAY);
}

void MapFrameProcessor::DFS(uint16_t x, uint16_t y, uint8_t component, int& num, cv::Mat& debug, std::shared_ptr<PlanarRegion> planarRegion)
{
   if (_visited(x, y))
      return;

   num++;
   _visited(x, y) = 1;
   _region(x, y) = component;
   cv::Vec6f patch = _frame.getRegionOutput().at<cv::Vec6f>(x, y);
   planarRegion->AddPatch(Eigen::Vector3f(patch[0], patch[1], patch[2]), Eigen::Vector3f(patch[3], patch[4], patch[5]));
   if (_app.SHOW_PATCHES)
      circle(debug, cv::Point((y) * PATCH_HEIGHT, (x) * PATCH_WIDTH), 2,
             cv::Scalar((component + 1) * 231 % 255, (component + 1) * 123 % 255, (component + 1) * 312 % 255), -1);

   int count = 0;
   for (int i = 0; i < 8; i++)
   {
      if (x + adx[i] < SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < SUB_W - 1 && y + ady[i] > 1)
      {
         uint8_t newPatch = _frame.patchData.at<uint8_t>((x + adx[i]), (y + ady[i]));
         if (newPatch == 255)
         {
            count++;
            DFS(x + adx[i], y + ady[i], component, num, debug, planarRegion);
         }
      }
   }
   if (count != 8)
   {
      _boundary(x, y) = 1;
      planarRegion->insertLeafPatch(Eigen::Vector2i(x, y));
      if (_app.SHOW_BOUNDARIES)
         circle(debug, cv::Point((y) * PATCH_HEIGHT, (x) * PATCH_WIDTH), 2, cv::Scalar(255, 255, 255), -1);
   }
   if (_app.VISUAL_DEPTH_DEBUG)
      DisplayDebugger(_app.VISUAL_DEBUG_DELAY);
}
