package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.ros2.ROS2Node;

import java.util.Collections;

public class ROS2BehaviorTreeUpdateThread extends RepeatingTaskThread
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2BehaviorTreeExecutor executor;

   public ROS2BehaviorTreeUpdateThread(ROS2Node ros2Node, DRCRobotModel robotModel, SceneGraph sceneGraph, DetectionManager detectionManager)
   {
      super(ROS2BehaviorTreeUpdateThread.class.getSimpleName());
      setFrequencyLimit(ROS2BehaviorTreeState.SYNC_FREQUENCY);

      ROS2ControllerHelper ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2ControllerHelper.getROS2NodeInterface());

      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
      referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      referenceFrameLibrary.addDynamicCollection(sceneGraph.asNewDynamicReferenceFrameCollection());

      executor = new ROS2BehaviorTreeExecutor(ros2ControllerHelper, robotModel, syncedRobot, referenceFrameLibrary, sceneGraph, detectionManager);
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
