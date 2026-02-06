package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;

public class ROS2BehaviorTreeUpdateThread extends RepeatingTaskThread
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2BehaviorTreeExecutor executor;

   public ROS2BehaviorTreeUpdateThread(ROS2Node ros2Node,
                                       ROS2PeerClockOffsetEstimator peerClockOffsetEstimator,
                                       DRCRobotModel robotModel,
                                       ImageSensor imageSensor,
                                       YOLOv8DetectionExecutor yolo,
                                       IsaacROSFoundationPoseCommunicatorMap foundationPose,
                                       TerrainMapData terrainMapData)
   {
      super(ROS2BehaviorTreeUpdateThread.class.getSimpleName());
      setFrequencyLimit(ROS2BehaviorTree.SYNC_FREQUENCY);

      ROS2ControllerHelper ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2ControllerHelper.getROS2Node());

      executor = new ROS2BehaviorTreeExecutor(ros2ControllerHelper, syncedRobot, imageSensor, yolo, foundationPose, terrainMapData, peerClockOffsetEstimator);
   }

   @Override
   protected synchronized void runTask()
   {
      syncedRobot.update();
      executor.update();
   }

   @Override
   public synchronized void kill()
   {
      super.kill();

      syncedRobot.destroy();
      executor.destroy();
   }
}
