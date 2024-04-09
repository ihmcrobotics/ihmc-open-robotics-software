package us.ihmc.behaviors.door;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final DoorTraversalState state;
   private final DoorTraversalDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;

   private final SideDependentList<RevoluteJoint> x1KnuckleJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> x2KnuckleJoints = new SideDependentList<>();

   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
   private boolean waitForPullScrewToFinish = false;

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

      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasSakeGripperJoints(side))
         {
            x1KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(0));
            x2KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(1));
         }
      }
   }

   @Override
   public void tick()
   {
      super.tick();

      // TODO: Tick children
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);

      if (state.isTreeStructureValid())
      {
         if (state.getStabilizeDetectionAction().getIsExecuting())
         {
            for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
            {
               if (sceneNode instanceof StaticRelativeSceneNode staticNode && staticNode.getName().contains("door"))
               {
                  staticNode.clearOffset();
                  staticNode.freeze();
               }
            }
         }
         // Here we are preventing the below logic from triggering more than once at a time
         if (!state.getPullScrewPrimitiveAction().getIsExecuting())
         {
            waitForPullScrewToFinish = false;
         }
         if (!waitForPullScrewToFinish && state.getPullScrewPrimitiveAction().getIsExecuting())
         {
            double knuckle1Q = x1KnuckleJoints.get(RobotSide.RIGHT).getQ();
            double knuckle2Q = x2KnuckleJoints.get(RobotSide.RIGHT).getQ();
            double handOpenAngle = SakeHandParameters.knuckleJointAnglesToHandOpenAngle(knuckle1Q, knuckle2Q);

            double lostGraspDetectionHandOpenAngle = getDefinition().getLostGraspDetectionHandOpenAngle().getValue();
            if (handOpenAngle < lostGraspDetectionHandOpenAngle)
            {
               state.getLogger().info("""
                                      Retrying pull door. Hand open angle %.2f%s / %.2f%s degrees.
                                      Stopping all trajectories.
                                      Going back to %s.
                                      """.formatted(Math.toDegrees(handOpenAngle),
                                                    EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                    Math.toDegrees(lostGraspDetectionHandOpenAngle),
                                                    EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                    state.getWaitToOpenRightHandAction().getDefinition().getName()));
               ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
               waitForPullScrewToFinish = true;
               state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
               state.getPullScrewPrimitiveAction().setIsExecuting(false);
            }
         }
      }
   }

   public void updateActionSubtree(BehaviorTreeNodeExecutor<?, ?> node)
   {
      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         if (child instanceof ActionNodeExecutor<?, ?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }
}
