package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.tools.FootstepPlannerLoggingTools;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionLoggingTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.file.Paths;

public class TerrainPlanningDatasetGenerator
{
   private PerceptionDataLogger perceptionDataLogger;

   public void configureLogger()
   {
      if (perceptionDataLogger == null)
      {
         perceptionDataLogger = new PerceptionDataLogger();
         String logFileName = HDF5Tools.generateLogFileName();
         FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

         perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
         perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.INTERNAL_HEIGHT_MAP_NAME);
         perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME);
         perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.SENSOR_CROPPED_HEIGHT_MAP_NAME);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.START_FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.START_FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.GOAL_FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.GOAL_FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.INITIAL_FOOTSTEP_SIDE, 1, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
         perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_SIDE, 1, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      }
   }

   public void stopLogging()
   {
      if (perceptionDataLogger != null)
      {
         perceptionDataLogger.closeLogFile();
         perceptionDataLogger = null;
      }
   }

   public void logInternalHeightMap(Mat heightMap, boolean heightMapCaptured)
   {
      if (!heightMapCaptured)
      {;
         PerceptionLoggingTools.logHeightMap(perceptionDataLogger, heightMap, PerceptionLoggerConstants.INTERNAL_HEIGHT_MAP_NAME);
         heightMapCaptured = true;
      }
   }

   public void logCompletePlan(FootstepPlannerOutput plannerOutput, Mat heightMap, RigidBodyTransform sensorTransform, SideDependentList<FramePose3D> startPose,
                               SideDependentList<FramePose3D> goalPose, boolean sidednessBit)
   {
      FootstepPlannerLoggingTools.printFootstepPlan(plannerOutput);
      FootstepPlannerLoggingTools.logFootsteps(plannerOutput,
                                               perceptionDataLogger,
                                               sensorTransform,
                                               startPose,
                                               goalPose,
                                               sidednessBit);
      PerceptionLoggingTools.logHeightMap(perceptionDataLogger,
                                          heightMap,
                                          PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME);
   }
}
