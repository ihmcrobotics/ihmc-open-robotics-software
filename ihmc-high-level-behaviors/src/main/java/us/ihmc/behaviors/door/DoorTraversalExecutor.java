package us.ihmc.behaviors.door;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final DoorTraversalState state;
   private final DoorTraversalDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;

   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
   private boolean waitForPullScrewToFinish = false;
   private boolean waitForGraspToFinish = false;

   public DoorTraversalExecutor(long id,
                                CRDTInfo crdtInfo,
                                WorkspaceResourceDirectory saveFileDirectory,
                                ROS2ControllerHelper ros2ControllerHelper,
                                ROS2SyncedRobotModel syncedRobot,
                                SceneGraph sceneGraph)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.sceneGraph = sceneGraph;
   }

   @Override
   public void tick()
   {
      super.tick();
   }

   @Override
   public void update()
   {
      super.update();
   }
}
