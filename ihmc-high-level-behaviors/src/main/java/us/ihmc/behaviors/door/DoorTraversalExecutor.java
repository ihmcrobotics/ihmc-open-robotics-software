package us.ihmc.behaviors.door;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalExecutor extends BehaviorTreeNodeExecutor<DoorTraversalState, DoorTraversalDefinition>
{
   private final DoorTraversalState state;
   private final DoorTraversalDefinition definition;
   private final ROS2SyncedRobotModel syncedRobot;

   private final SideDependentList<RevoluteJoint> x1KnuckleJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> x2KnuckleJoints = new SideDependentList<>();

   public DoorTraversalExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.syncedRobot = syncedRobot;

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
         if (state.getPullScrewPrimitiveAction().getIsExecuting())
         {
            double knuckle1Q = x1KnuckleJoints.get(RobotSide.RIGHT).getQ();
            double knuckle2Q = x2KnuckleJoints.get(RobotSide.RIGHT).getQ();
            double handOpenAngle = SakeHandParameters.knuckleJointAnglesToHandOpenAngle(knuckle1Q, knuckle2Q);

            if (handOpenAngle < Math.toRadians(20.0)) // TODO: Tune
            {
               state.getActionSequence().setExecutionNextIndex(state.getWaitToOpenRightHandAction().getActionIndex());
               state.getRetryingPullDoorNotification().set();
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
