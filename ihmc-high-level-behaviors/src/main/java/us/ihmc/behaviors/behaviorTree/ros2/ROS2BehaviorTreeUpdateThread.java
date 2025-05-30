package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RapidHeightMapThread;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

import java.util.Collections;

public class ROS2BehaviorTreeUpdateThread extends RepeatingTaskThread
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2BehaviorTreeExecutor executor;

   public ROS2BehaviorTreeUpdateThread(ROS2Node ros2Node,
                                       ROS2PeerClockOffsetEstimator peerClockOffsetEstimator,
                                       DRCRobotModel robotModel,
                                       SceneGraph sceneGraph,
                                       DetectionManager detectionManager,
                                       RapidHeightMapThread rapidHeightMapUpdateThread)
   {
      super(ROS2BehaviorTreeUpdateThread.class.getSimpleName());
      setFrequencyLimit(ROS2BehaviorTree.SYNC_FREQUENCY);

      ROS2ControllerHelper ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2ControllerHelper.getROS2Node());

      ReferenceFrameLibrary referenceFrameLibrary = new ReferenceFrameLibrary();
      referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
      referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      referenceFrameLibrary.addDynamicCollection(sceneGraph.asNewDynamicReferenceFrameCollection());
      for (RobotSide side: RobotSide.values)
      {
         referenceFrameLibrary.addAll(Collections.singleton(syncedRobot.getReferenceFrames().getHandZUpFrame(side)));
      }

      executor = new ROS2BehaviorTreeExecutor(ros2ControllerHelper,
                                              robotModel,
                                              syncedRobot,
                                              peerClockOffsetEstimator,
                                              referenceFrameLibrary,
                                              sceneGraph,
                                              detectionManager,
                                              rapidHeightMapUpdateThread);
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
