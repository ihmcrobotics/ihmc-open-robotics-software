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
   private static final String SLAM_MODULE_CONFIGURATION_FILE_NAME = "./Configurations/atlasSLAMModuleConfiguration.txt";

   private final Path slamConfigurationFilePath;
   private final DRCRobotModel robotModel;

   public AtlasPerceptionSuite(Messager messager)
   {
      super(messager);

      Path slamConfigurationFilePath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software/atlas");
      slamConfigurationFilePath = Paths.get(slamConfigurationFilePath.toString(), "/src/main/resources/" + SLAM_MODULE_CONFIGURATION_FILE_NAME);

      this.slamConfigurationFilePath = slamConfigurationFilePath;

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
   }

   @Override
   protected SLAMModule createSLAMModule(Messager messager)
   {
      return AtlasSLAMModule.createIntraprocessModule(ros2Node, robotModel, messager, slamConfigurationFilePath);
   }

   public static AtlasPerceptionSuite createIntraprocess(Messager messager)
   {
      return new AtlasPerceptionSuite(messager);
   }

}
