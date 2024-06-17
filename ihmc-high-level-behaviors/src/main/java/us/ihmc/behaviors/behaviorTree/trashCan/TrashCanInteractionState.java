package us.ihmc.behaviors.behaviorTree.trashCan;

import behavior_msgs.msg.dds.DoorTraversalStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class TrashCanInteractionState extends BehaviorTreeNodeState<TrashCanInteractionDefinition>
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

   private final CRDTUnidirectionalBoolean isBlockingPathToDoor;
   private final CRDTUnidirectionalDouble distanceFromDoor;
   private final CRDTUnidirectionalDouble distanceFromCouch;

   public TrashCanInteractionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new TrashCanInteractionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      isBlockingPathToDoor = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.ROBOT, crdtInfo, false);
      distanceFromDoor = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      distanceFromCouch = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
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

   public void toMessage(TrashCanInteractionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setDistanceFromDoor(distanceFromDoor.toMessage());
      message.setDistanceFromCouch(distanceFromCouch.toMessage());
   }

   public void fromMessage(TrashCanInteractionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      distanceFromDoor.fromMessage(message.getDistanceFromDoor());
      distanceFromCouch.fromMessage(message.getDistanceFromCouch());
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

   public CRDTUnidirectionalBoolean getIsBlockingPathToDoor()
   {
      return isBlockingPathToDoor;
   }

   public CRDTUnidirectionalDouble getDistanceFromDoor()
   {
      return distanceFromDoor;
   }

   public CRDTUnidirectionalDouble getDistanceFromCouch()
   {
      return distanceFromCouch;
   }
}
