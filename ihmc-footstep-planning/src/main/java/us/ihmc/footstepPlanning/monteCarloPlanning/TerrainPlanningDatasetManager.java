package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.tools.FootstepPlannerLoggingTools;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionLoggingTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class TerrainPlanningDatasetManager
{
   private ArrayList<SideDependentList<Pose3D>> footstepPairList = new ArrayList<>();
   private PerceptionDataLogger perceptionDataLogger;
   private Path path = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY;
   private List<DataSet> dataSets;

   public void setPath(Path path)
   {
      this.path = path;
   }

   public void loadDataSets()
   {
      dataSets = DataSetIOTools.loadDataSets(DataSet::hasPlannerInput);
   }

   public void configureLogger(String suffix)
   {
      if (perceptionDataLogger == null)
      {
         startLogger(suffix);
      }
   }

   public void startLogger(String suffix)
   {
      perceptionDataLogger = new PerceptionDataLogger();
      String logFileName = HDF5Tools.generateFileName(suffix);
      FileTools.ensureDirectoryExists(path, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      perceptionDataLogger.openLogFile(path.resolve(logFileName).toString());
      perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.INTERNAL_HEIGHT_MAP_NAME);
      perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME);
      perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.SENSOR_CROPPED_HEIGHT_MAP_NAME);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.START_FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.START_FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.GOAL_FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.GOAL_FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.DEPTH_SENSOR_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.DEPTH_SENSOR_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.INITIAL_FOOTSTEP_SIDE, 1, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_SIDE, 1, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
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
      //FootstepPlannerLoggingTools.printFootstepPlan(plannerOutput);
      FootstepPlannerLoggingTools.logFootsteps(plannerOutput,
                                               perceptionDataLogger,
                                               sensorTransform,
                                               startPose,
                                               goalPose,
                                               sidednessBit);
      PerceptionLoggingTools.logHeightMap(perceptionDataLogger,
                                          heightMap,
                                          PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME);

      //perceptionDataLogger.printStats();
   }

   public void loadRequests(File directory, int totalLogs)
   {
      footstepPairList.clear();
      File requests = new File(directory.getAbsolutePath() + "/processed_requests.txt");
      try (Scanner scanner = new Scanner(requests))
      {
         for (int k = 0; k<totalLogs; k++)
         {
            LogTools.warn("Loading log: {}", k);

            String line = scanner.nextLine();
            LogTools.info("FIRST: {}", line);
            line = scanner.nextLine();
            LogTools.info("SECOND: {}", line);
            if (line.contains("Loading log"))
            {
               line = scanner.nextLine();
               LogTools.info("THIRD: {}", line);

               for (int i = 0; i<10; i++)
               {
                  LogTools.warn("Loading pair: {}", i);
                  SideDependentList<Pose3D> posePair = new SideDependentList<>(new Pose3D(), new Pose3D());
                  footstepPairList.add(posePair);

                  line = scanner.nextLine();
                  line = line.replace(" ", "");
                  LogTools.info("INPUT: {}", line);
                  posePair.put(RobotSide.RIGHT, loadPoseFromString(line));

                  line = scanner.nextLine();
                  line = line.replace(" ", "");
                  LogTools.info("INPUT: {}", line);
                  posePair.put(RobotSide.LEFT, loadPoseFromString(line));
               }
            }
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      printAllPairs();
   }

   public void printAllPairs()
   {
      for (int i = 0; i<footstepPairList.size(); i++)
      {
         LogTools.info("---------------------------------------------- Pair: {} ---------------------------------------------------------", i);
         LogTools.info("Left: {}", footstepPairList.get(i).get(RobotSide.LEFT));
         LogTools.info("Right: {}", footstepPairList.get(i).get(RobotSide.RIGHT));
         LogTools.info("------------------------------------------------------------------------------------------------------------------");
      }
   }

   public Pose3D loadPoseFromString(String logLine)
   {
      String valueString = logLine.split("Pose:")[1];


      String[] poseSplit = valueString.split("x");

      LogTools.info("Value String: {} -> ({}) ({})", valueString, poseSplit[0], poseSplit[1]);

      String positionString = poseSplit[0];
      String orientationString = poseSplit[1];

      LogTools.info("Position: {}, Orientation: {}", positionString, orientationString);

      String[] positionSplit = positionString.split(",");
      String[] orientationSplit = orientationString.split(",");

      LogTools.info("Position: {}", Arrays.toString(positionSplit));
      LogTools.info("Orientation: {}", Arrays.toString(orientationSplit));

      double x = Double.parseDouble(positionSplit[0]);
      double y = Double.parseDouble(positionSplit[1]);
      double z = Double.parseDouble(positionSplit[2]);

      double qx = Double.parseDouble(orientationSplit[0]);
      double qy = Double.parseDouble(orientationSplit[1]);
      double qz = Double.parseDouble(orientationSplit[2]);
      double qw = Double.parseDouble(orientationSplit[3]);

      LogTools.info(String.format("[FINAL]: Position: (%f, %f, %f), Orientation: (%f, %f, %f, %f)", x, y, z, qx, qy, qz, qw));

      return new Pose3D(new Point3D(x, y, z), new Quaternion(qx, qy, qz, qw));
   }

   public HeightMapMessage getHeightMapFromDataset(int index)
   {
      PlanarRegionsList regions = dataSets.get(index).getPlanarRegionsList();
      HeightMapMessage message = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(regions);
      return message;
   }

   public ArrayList<SideDependentList<Pose3D>> getFootstepPairList()
   {
      return footstepPairList;
   }

   public SideDependentList<Pose3D> getFootstepPair(int index)
   {
      return footstepPairList.get(index);
   }

   public List<DataSet> getDataSets()
   {
      return dataSets;
   }

   public DataSet getDataSet(int index)
   {
      return dataSets.get(index);
   }
}
