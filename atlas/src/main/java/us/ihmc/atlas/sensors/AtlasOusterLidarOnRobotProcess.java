package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.ouster.OusterDriverAndDepthPublisher;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;

public class AtlasOusterLidarOnRobotProcess
{
   private final ROS2SyncedRobotModel syncedRobot;

   public AtlasOusterLidarOnRobotProcess()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "atlas_ouster_driver_and_depth_publisher");

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      ROS2ControllerHelper controllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         ros2Node.destroy();
      }, getClass().getSimpleName() + "Shutdown"));

      new OusterDriverAndDepthPublisher(controllerHelper, this::sensorFrameUpdater, PerceptionAPI.OUSTER_DEPTH_IMAGE, PerceptionAPI.OUSTER_LIDAR_SCAN);
   }

   private HumanoidReferenceFrames sensorFrameUpdater()
   {
      syncedRobot.update();
      return syncedRobot.getReferenceFrames();
   }

   public static void main(String[] args)
   {
      new AtlasOusterLidarOnRobotProcess();
   }
}
