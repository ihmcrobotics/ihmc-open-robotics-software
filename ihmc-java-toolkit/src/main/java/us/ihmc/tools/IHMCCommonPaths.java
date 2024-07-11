package us.ihmc.tools;

import java.nio.file.Path;
import java.nio.file.Paths;

public class IHMCCommonPaths
{
   private static final String USER_HOME_STRING = System.getProperty("user.home");
   public static final Path USER_HOME_DIRECTORY = Paths.get(USER_HOME_STRING);

   public static final String DOT_IHMC_DIRECTORY_NAME = ".ihmc";
   public static final Path DOT_IHMC_DIRECTORY = USER_HOME_DIRECTORY.resolve(DOT_IHMC_DIRECTORY_NAME);

   public static final String LOGS_DIRECTORY_NAME = "logs";
   public static final Path LOGS_DIRECTORY = DOT_IHMC_DIRECTORY.resolve(LOGS_DIRECTORY_NAME);

   public static final String PERCEPTION_LOGS_DIRECTORY_NAME = "perception";
   public static final Path PERCEPTION_LOGS_DIRECTORY = LOGS_DIRECTORY.resolve(PERCEPTION_LOGS_DIRECTORY_NAME);

   public static final String VR_DIRECTORY_NAME = "vr";
   public static final Path VR_DIRECTORY = DOT_IHMC_DIRECTORY.resolve(VR_DIRECTORY_NAME);

   public static final String ASTAR_FOOTSTEP_PLANNER_DIRECTORY_NAME = "astar_footstep_planner";
   public static final Path ASTAR_FOOTSTEP_PLANNER_DIRECTORY = LOGS_DIRECTORY.resolve(ASTAR_FOOTSTEP_PLANNER_DIRECTORY_NAME);

   public static final String MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY_NAME = "monte-carlo-footstep-planner";
   public static final Path MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY = LOGS_DIRECTORY.resolve(MONTE_CARLO_FOOTSTEP_PLANNER_DIRECTORY_NAME);

   public static final String PLANNING_DATASETS_DIRECTORY_NAME = "planning-datasets";
   public static final Path PLANNING_DATASETS_DIRECTORY = LOGS_DIRECTORY.resolve(PLANNING_DATASETS_DIRECTORY_NAME);

   public static final String ELEVATION_DATASETS_DIRECTORY_NAME = "elevation-maps";
   public static final Path ELEVATION_DATASETS_DIRECTORY = PLANNING_DATASETS_DIRECTORY.resolve(ELEVATION_DATASETS_DIRECTORY_NAME);

   public static final String CONTINUOUS_HIKING_DIRECTORY_NAME = "continuous-hiking";
   public static final Path CONTINUOUS_HIKING_DIRECTORY = LOGS_DIRECTORY.resolve(CONTINUOUS_HIKING_DIRECTORY_NAME);

   public static final String TERRAIN_MAP_DIRECTORY_NAME = "terrain-map";
   public static final Path TERRAIN_MAP_DIRECTORY = LOGS_DIRECTORY.resolve(TERRAIN_MAP_DIRECTORY_NAME);

   public static final String MISSION_CONTROL_LOGS_DIRECTORY_NAME = "mission-control";
   public static final Path MISSION_CONTROL_LOGS_DIRECTORY = LOGS_DIRECTORY.resolve(MISSION_CONTROL_LOGS_DIRECTORY_NAME);

   private static final String YOLO_MODELS_DIRECTORY_NAME = "yolo-models";
   public static final Path YOLO_MODELS_DIRECTORY = DOT_IHMC_DIRECTORY.resolve(YOLO_MODELS_DIRECTORY_NAME);
}
