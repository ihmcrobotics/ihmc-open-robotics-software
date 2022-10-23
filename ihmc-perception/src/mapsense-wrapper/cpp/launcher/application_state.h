#pragma once

#include <sstream>
#include <fstream>

class ApplicationState
{
   public:
      ApplicationState();

      void Update();

   public:
      float MERGE_DISTANCE_THRESHOLD = 0.016;
      float MERGE_ANGULAR_THRESHOLD =  0.82;
      float HASH_MERGE_DISTANCE_THRESHOLD = 0.1;
      float HASH_MERGE_ANGULAR_THRESHOLD = 0.7;

      bool FILTER_SELECTED = false;
      float FILTER_DISPARITY_THRESHOLD = 2000;
      float MAGNUM_PATCH_SCALE = 0.007;

      int REGION_MIN_PATCHES = 20;
      int REGION_BOUNDARY_DIFF = 20;

      int HASH_THREAD_NUM = 2;

      /*
       * NOTE: The following parameters should meet these requirements.
       * a) InputHeight should be divisible by (KernelResLevel * AspectRatioHeight)
       * b) InputWidth should be divisible by (KernelResLevel * AspectRatioWidth)
       * */
      bool HASH_DIFFUSION_ENABLED = false;
      bool HASH_PRINT_MAT = false;
      bool HASH_PARTS_ENABLED = false;
      int REGION_MODE = 0; // 0 for depth, 1 for point-cloud
      int HASH_INPUT_HEIGHT = 64;
      int HASH_INPUT_WIDTH = 1024;
      int DEPTH_INPUT_HEIGHT = 480;
      int DEPTH_INPUT_WIDTH = 640;
      int HASH_ADJ_SKIPS = 2;

      int KERNEL_SLIDER_LEVEL = 2;
      int HASH_PATCH_HEIGHT =  2;
      int HASH_PATCH_WIDTH =   4;
      int DEPTH_PATCH_HEIGHT = KERNEL_SLIDER_LEVEL;
      int DEPTH_PATCH_WIDTH = KERNEL_SLIDER_LEVEL;
      int SUB_H = DEPTH_INPUT_HEIGHT / DEPTH_PATCH_HEIGHT;
      int SUB_W = DEPTH_INPUT_WIDTH / DEPTH_PATCH_WIDTH;

      int HASH_SUB_H = HASH_INPUT_HEIGHT / HASH_PATCH_HEIGHT;
      int HASH_SUB_W = HASH_INPUT_WIDTH / HASH_PATCH_WIDTH;

      int FILTER_KERNEL_SIZE = 4;
      int FILTER_SUB_H = DEPTH_INPUT_HEIGHT / FILTER_KERNEL_SIZE;
      int FILTER_SUB_W = DEPTH_INPUT_WIDTH / FILTER_KERNEL_SIZE;

      //    float DEPTH_FX = 459.97;
      //    float DEPTH_FY = 459.80;
      //    float DEPTH_CX = 341.84;
      //    float DEPTH_CY = 249.17;

      int DIVISION_FACTOR = 1; // To simulate lower resolution by a factor.

      float DEPTH_FX = 0;
      float DEPTH_FY = 0;
      float DEPTH_CX = 0;
      float DEPTH_CY = 0;

      int NUM_SKIP_EDGES = 8;
      int VISUAL_DEBUG_DELAY = 10;

      bool SHOW_BOUNDARIES = false;
      bool SHOW_PATCHES = true;
      bool VISUAL_DEPTH_DEBUG = false;
      bool VISUAL_HASH_DEBUG = false;
      bool SHOW_INPUT_COLOR = false;
      bool SHOW_INPUT_DEPTH = false;
      bool SHOW_FILTERED_DEPTH = false;
      bool SHOW_HASH_REGION_COMPONENTS = false;
      bool SHOW_DEPTH_REGION_COMPONENTS = false;
      bool SHOW_STEREO_LEFT = false;
      bool SHOW_STEREO_RIGHT = false;

      bool SHOW_HASH_NX = false;
      bool SHOW_HASH_NY = false;
      bool SHOW_HASH_NZ = false;
      bool SHOW_HASH_GX = false;
      bool SHOW_HASH_GY = false;
      bool SHOW_HASH_GZ = false;

      bool USE_LINE_MESH = false;

      bool SHOW_GRAPH = true;
      bool ROS_ENABLED = true;
      bool SHOW_REGION_EDGES = false;

      bool STEREO_DRIVER = false;
      bool DEPTH_ALIGNED = false;
      bool EARLY_GAUSSIAN_BLUR = true;

      bool ICP_ODOMETRY_ENABLED = false;
      bool STEREO_ODOMETRY_ENABLED = false;
      bool DEPTH_PLANAR_REGIONS_ENABLED = false;
      bool HASH_PLANAR_REGIONS_ENABLED = false;
      bool SLAM_ENABLED = false;
      bool DATASET_ENABLED = false;
      bool FACTOR_GRAPH_ENABLED = false;

      std::string TOPIC_CAMERA_NAME = "chest_l515";

      int GAUSSIAN_SIZE = 4;
      int  GAUSSIAN_SIGMA = 20;

      /*  VISUALIZATION-ONLY */
      float DISPLAY_WINDOW_SIZE = 1.5f;
      float DEPTH_BRIGHTNESS = 40;
      float DEPTH_DISPLAY_OFFSET = 100;

      bool EXPORT_REGIONS = false;
      bool GENERATE_REGIONS = true;

      float REGION_GROWTH_FACTOR = 0.01;

      /* Stereo Matching Parameters */
      int STEREO_NUM_DISPARITIES = 1;
      int STEREO_BLOCK_SIZE = 2;
      int STEREO_PRE_FILTER_SIZE = 2;
      int STEREO_PRE_FILTER_TYPE = 1;
      int STEREO_PRE_FILTER_CAP = 31;
      int STEREO_MIN_DISPARITY = 0;
      int STEREO_TEXTURE_THRESHOLD = 10;
      int STEREO_UNIQUENESS_RATIO = 15;
      int STEREO_SPECKLE_RANGE = 0;
      int STEREO_SPECKLE_WINDOW_SIZE = 0;
      int STEREO_DISP_12_MAX_DIFF = -1;

      std::string OUSTER_POINTS = "/os_cloud_node/points";
      std::string ZED_LEFT_IMAGE_RAW = "/zed/color/left/image_raw";
      std::string ZED_RIGHT_IMAGE_RAW = "/zed/color/right/image_raw";

      std::string KITTI_LEFT_IMG_RECT = "/kitti/left/image_rect/compressed";
      std::string KITTI_RIGHT_IMG_RECT = "/kitti/right/image_rect/compressed";
      std::string KITTI_LIDAR_POINTS = "/kitti/lidar/points";

      std::string L515_COLOR = "/camera/color/image_raw/compressed";
      std::string L515_DEPTH = "/camera/aligned_depth_to_color/image_raw";
      std::string L515_DEPTH_INFO = "/camera/aligned_depth_to_color/camera_info";

      std::string L515_ALIGNED_DEPTH = "/camera/aligned_depth_to_color/image_raw";
      std::string L515_ALIGNED_DEPTH_INFO = "/camera/aligned_depth_to_color/camera_info";

      std::string BLACKFLY_RIGHT_RAW = "/blackfly/right/image_color";

      float COMPRESS_COSINE_THRESHOLD = 0.619f;
      float COMPRESS_DIST_THRESHOLD = 0.011f;
      float SEGMENT_DIST_THRESHOLD = 0.233f;
      float MATCH_DIST_THRESHOLD = 0.1f;
      float MATCH_ANGULAR_THRESHOLD = 0.9f;
      float MATCH_IOU_THRESHOLD = 0.1f;
      int MATCH_PERCENT_VERTEX_THRESHOLD = 20;


};

