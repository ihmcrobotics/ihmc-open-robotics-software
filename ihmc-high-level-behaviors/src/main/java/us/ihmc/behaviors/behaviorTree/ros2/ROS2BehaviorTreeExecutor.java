package us.ihmc.behaviors.behaviorTree.ros2;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutor;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

/**
 * Top level class for the robot's behavior tree.
 */
public class ROS2BehaviorTreeExecutor extends BehaviorTreeExecutor
{
   private final ROS2BehaviorTreeState ros2BehaviorTreeState;
   private final ROS2Heartbeat arUcoDemandHeartbeat;

   public ROS2BehaviorTreeExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                   DRCRobotModel robotModel,
                                   ROS2SyncedRobotModel syncedRobot,
                                   ReferenceFrameLibrary referenceFrameLibrary,
                                   SceneGraph sceneGraph)
   {
      super(robotModel, syncedRobot, referenceFrameLibrary, sceneGraph, ros2ControllerHelper);

      ros2BehaviorTreeState = new ROS2BehaviorTreeState(getState(), this::setRootNode, ros2ControllerHelper);
      arUcoDemandHeartbeat = new ROS2Heartbeat(ros2ControllerHelper, PerceptionAPI.REQUEST_ARUCO);
   }

   public void update()
   {
      ros2BehaviorTreeState.updateSubscription();

      super.update();
      arUcoDemandHeartbeat.setAlive(getRootNode() != null);

      ros2BehaviorTreeState.updatePublication();
   }

   public void destroy()
   {
      ros2BehaviorTreeState.destroy();

      super.destroy();
   }
}
