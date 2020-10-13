package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMModule;
import us.ihmc.robotEnvironmentAwareness.perceptionSuite.PerceptionSuite;
import us.ihmc.tools.io.WorkspacePathTools;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

public class AtlasPerceptionSuite extends PerceptionSuite
{
   private static final Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
   private static final String directory = "/atlas/src/main/resources/";

   private static final String SLAM_MODULE_CONFIGURATION_FILE_NAME = "atlasSLAMModuleConfiguration.txt";
   private static final String SEGMENTATION_MODULE_CONFIGURATION_FILE_NAME = "atlasSegmentationModuleConfiguration.txt";
   private static final String LIDAR_REA_MODULE_CONFIGURATION_FILE_NAME = "atlasREAModuleConfiguration.txt";
   private static final String REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME = "atlasRealSenseREAModuleConfiguration.txt";

   private final Path slamConfigurationFilePath;
   private final DRCRobotModel robotModel;

   public AtlasPerceptionSuite(Messager messager)
   {
      super(messager,
            Paths.get(rootPath.toString(), directory + SEGMENTATION_MODULE_CONFIGURATION_FILE_NAME),
            Paths.get(rootPath.toString(), directory + LIDAR_REA_MODULE_CONFIGURATION_FILE_NAME),
            Paths.get(rootPath.toString(), directory + REALSENSE_REA_MODULE_CONFIGURATION_FILE_NAME),
            "atlas");

      this.slamConfigurationFilePath = Paths.get(rootPath.toString(), directory + SLAM_MODULE_CONFIGURATION_FILE_NAME);

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
   }

   @Override
   protected SLAMModule createSLAMModule(Messager messager)
   {
      return AtlasSLAMModule.createIntraprocessModule(ros2Node, robotModel, messager, slamConfigurationFilePath.toFile());
   }

   public static AtlasPerceptionSuite createIntraprocess(Messager messager)
   {
      return new AtlasPerceptionSuite(messager);
   }

}
