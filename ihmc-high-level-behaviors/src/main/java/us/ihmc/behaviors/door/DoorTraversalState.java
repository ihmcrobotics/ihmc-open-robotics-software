package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalState extends BehaviorTreeNodeState<DoorTraversalDefinition>
{
   public static final String STABILIZE_DETECTION = "Stabilize Detection";
   public static final String WAIT_TO_OPEN_RIGHT_HAND = "Wait to open right hand";
   public static final String PULL_SCREW_PRIMITIVE = "Pull Screw primitive";
   public static final String POST_PULL_DOOR = "Post pull door evaluation";
   public static final String POST_GRASP_HANDLE = "Evaluate grasp";

   private ActionSequenceState actionSequence;
   private WaitDurationActionState stabilizeDetectionAction;
   private WaitDurationActionState waitToOpenRightHandAction;
   private ScrewPrimitiveActionState pullScrewPrimitiveAction;
   private WaitDurationActionState postGraspEvaluationAction;
   private WaitDurationActionState postPullDoorEvaluationAction;

   private final CRDTUnidirectionalDouble doorHingeJointAngle;
   private final CRDTUnidirectionalDouble doorHandleDistanceFromStart;

   public DoorTraversalState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new DoorTraversalDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      doorHingeJointAngle = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      doorHandleDistanceFromStart = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, 0.0);
   }

   @Override
   public void update()
   {
      super.update();

      actionSequence = BehaviorTreeTools.findActionSequenceAncestor(this);

      updateActionSubtree(this);
   }

   public void updateActionSubtree(BehaviorTreeNodeState<?> node)
   {
      stabilizeDetectionAction = null;
      waitToOpenRightHandAction = null;
      pullScrewPrimitiveAction = null;
      postGraspEvaluationAction = null;
      postPullDoorEvaluationAction = null;

      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(STABILIZE_DETECTION))
            {
               stabilizeDetectionAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(WAIT_TO_OPEN_RIGHT_HAND))
            {
               waitToOpenRightHandAction = waitDurationAction;
            }
            if (actionNode instanceof ScrewPrimitiveActionState screwPrimitiveAction
                && screwPrimitiveAction.getDefinition().getName().equals(PULL_SCREW_PRIMITIVE))
            {
               pullScrewPrimitiveAction = screwPrimitiveAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(POST_GRASP_HANDLE))
            {
               postGraspEvaluationAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(POST_PULL_DOOR))
            {
               postPullDoorEvaluationAction = waitDurationAction;
            }
         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   public void toMessage(DoorTraversalStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setDoorHingeJointAngle(doorHingeJointAngle.toMessage());
      message.setDoorHandleDistanceFromStart(doorHandleDistanceFromStart.toMessage());
   }

   public void fromMessage(DoorTraversalStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      doorHingeJointAngle.fromMessage(message.getDoorHingeJointAngle());
      doorHandleDistanceFromStart.fromMessage(message.getDoorHandleDistanceFromStart());
   }

   public boolean arePullRetryNodesPresent()
   {
      boolean isValid = actionSequence != null;
      isValid &= waitToOpenRightHandAction != null;
      isValid &= pullScrewPrimitiveAction != null;
      isValid &= postGraspEvaluationAction != null;
      isValid &= postPullDoorEvaluationAction != null;
      return isValid;
   }

   public ActionSequenceState getActionSequence()
   {
      return actionSequence;
   }

   public WaitDurationActionState getStabilizeDetectionAction()
   {
      return stabilizeDetectionAction;
   }

   public WaitDurationActionState getWaitToOpenRightHandAction()
   {
      return waitToOpenRightHandAction;
   }

   public ScrewPrimitiveActionState getPullScrewPrimitiveAction()
   {
      return pullScrewPrimitiveAction;
   }

   public WaitDurationActionState getPostGraspEvaluationAction()
   {
      return postGraspEvaluationAction;
   }

   public WaitDurationActionState getPostPullDoorEvaluationAction()
   {
      return postPullDoorEvaluationAction;
   }

   public CRDTUnidirectionalDouble getDoorHingeJointAngle()
   {
      return doorHingeJointAngle;
   }

   public CRDTUnidirectionalDouble getDoorHandleDistanceFromStart()
   {
      return doorHandleDistanceFromStart;
   }
}
