package us.ihmc.atlas.sensors;

import org.bytedeco.javacv.FrameFilter.Exception;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.PlanarSegmentationModule;
import us.ihmc.ros2.Ros2Node;

import java.io.IOException;

public class AtlasPerceptionSuite
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private final AtlasSLAMModule slamModule;
   private final LIDARBasedREAModule reaModule;
   private final PlanarSegmentationModule segmentationModule;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);
   // TODO module for combining rea and segmentation

   public AtlasPerceptionSuite() throws IOException
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);

      slamModule = AtlasSLAMModule.createIntraprocessModule(ros2Node, drcRobotModel, MODULE_CONFIGURATION_FILE_NAME);
      reaModule = LIDARBasedREAModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME, ros2Node);
      segmentationModule = PlanarSegmentationModule.createIntraprocessModule(MODULE_CONFIGURATION_FILE_NAME, ros2Node);

      slamModule.attachOcTreeConsumer(segmentationModule);
   }

   public void start()
   {
      slamModule.start();
      reaModule.start();
      segmentationModule.start();
   }


   public void stop()
   {
      slamModule.stop();
      reaModule.stop();
      segmentationModule.stop();
      ros2Node.destroy();
   }
}
