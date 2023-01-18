package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.ouster.OusterDriverAndDepthPublisher;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;

public class AtlasOusterLidarOnRobotProcess
{
   private final ROS2SyncedRobotModel syncedRobot;

   public AtlasOusterLidarOnRobotProcess()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);

      RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_image_node");

      syncedRobot = new ROS2SyncedRobotModel(robotModel, realtimeROS2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      new OusterDriverAndDepthPublisher(realtimeROS2Node, this::sensorFrameUpdater, ROS2Tools.OUSTER_DEPTH_IMAGE);
   }

   private ReferenceFrame sensorFrameUpdater()
   {
      syncedRobot.update();
      return syncedRobot.getReferenceFrames().getOusterLidarFrame();
   }

   public static void main(String[] args)
   {
      new AtlasOusterLidarOnRobotProcess();
   }
}
