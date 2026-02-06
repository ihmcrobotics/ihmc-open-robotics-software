package us.ihmc.behaviors.behaviorTree.control.ai2r;

import behavior_msgs.msg.dds.AI2RCommandMessage;
import behavior_msgs.msg.dds.AI2RNavigationMessage;
import behavior_msgs.msg.dds.AI2RPickUpObjectMessage;
import behavior_msgs.msg.dds.AI2RReceiveObjectMessage;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionFootstepState;
import us.ihmc.behaviors.behaviorTree.action.actions.FootstepPlanActionState;
import us.ihmc.behaviors.behaviorTree.action.actions.HandPoseActionState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * AI2RSkillEditor is responsible for adapting and updating skill behaviors in the AI2R framework.
 * <p>
 * This class modifies behavior sequences dynamically based on incoming AI2R command messages, allowing for real-time adaptation of
 * robot skills such as navigation ("Go To"), object reception, and object pickup.
 * </p>
 * <p>
 * It processes commands by updating the relevant action states within a behavior tree or sequence, adjusting reference frames,
 * spatial relations, and object grasp information accordingly.
 * </p>
 * <p>
 * Key responsibilities include:
 * <ul>
 *     <li>Adapting navigation goals by changing parent frames and adjusting goal stance and focal points based on spatial relations.</li>
 *     <li>Updating object reception behaviors by setting proximity checks relative to the grasped object and robot hand side.</li>
 *     <li>Adjusting pickup behaviors by updating hand pose targets and grasped object names.</li>
 * </ul>
 * </p>
 * <p>
 * This class serves as a bridge between high-level AI2R command messages and low-level behavior execution states,
 * facilitating flexible and context-aware robot skill execution.
 * </p>
 * <p>
 * <b>Note:</b> Some functionality (e.g., pick-and-place adaptation) is currently commented out and may be implemented in future revisions.
 * </p>
 */
public class AI2RSkillEditor
{
   public enum SpatialRelationType
   {
      DEFAULT,
      FRONT,
      BEHIND,
      LEFT,
      RIGHT;

      public static final AI2RSkillEditor.SpatialRelationType[] values = values();
   }
   private String objectGrasped = "";
   private RobotSide graspSide = RobotSide.RIGHT;
   private int commandedBehaviorIndex;

   public void adaptSkills(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message, int commandedBehaviorIndex)
   {
      this.commandedBehaviorIndex = commandedBehaviorIndex;
      updateGoTo(behaviorToExecuteName, state, message);
      updateReceiveObject(behaviorToExecuteName, state, message);
      updatePickUpObject(behaviorToExecuteName, state, message);
//      updatePickAndPlace(behaviorToExecuteName, state, message);
   }

   private void updateGoTo(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message)
   {
      if (behaviorToExecuteName.contains("GOTO") && message.getAdaptingBehavior())
      {
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf.getDefinition().getName().toLowerCase().contains("go to action") && leaf instanceof FootstepPlanActionState gotoActionState)
            {
               AI2RNavigationMessage navigationMessage = message.getNavigation();
               String referenceFrameName = navigationMessage.getTargetObjectAsString();
               changeParentFrameGoToNode(gotoActionState.getDefinition(), gotoActionState, referenceFrameName);
               ReferenceFrame referenceFrame = gotoActionState.getFrameByName(referenceFrameName);

               String secondaryFrameName = navigationMessage.getPovObjectAsString();
               LogTools.warn(secondaryFrameName);
               ReferenceFrame secondaryFrame = gotoActionState.getFrameByName(secondaryFrameName);

               FramePoint3D goalStancePoint = new FramePoint3D(gotoActionState.getParentFrame());
               goalStancePoint.changeFrame(referenceFrame);
               goalStancePoint.setToZero();
               goalStancePoint.changeFrame(ReferenceFrame.getWorldFrame());
               double distanceToReferenceFrame = navigationMessage.getDistanceToObject();

               FramePoint3D goalFocalPoint = new FramePoint3D(gotoActionState.getParentFrame());
               goalFocalPoint.changeFrame(secondaryFrame);
               goalFocalPoint.setToZero();
               goalFocalPoint.changeFrame(ReferenceFrame.getWorldFrame());

               SpatialRelationType spatialRelationType = SpatialRelationType.values()[navigationMessage.getSpatialRelation()];
               Vector3D direction = new Vector3D();
               direction.sub(goalFocalPoint, goalStancePoint);

               // Only move if points are distinct
               if (direction.norm() > 1e-5)
               {
                  direction.normalize();
                  direction.scale(distanceToReferenceFrame);
                  switch (spatialRelationType)
                  {
                     case DEFAULT ->
                     {
                        direction.scale(distanceToReferenceFrame);
                        goalStancePoint.add(direction);
                     }
                     case FRONT ->
                     {
                        goalStancePoint.add(direction);
                     }
                     case BEHIND -> goalStancePoint.sub(direction);
                     case LEFT ->
                     {
                        // Get world up vector (Z-axis)
                        Vector3D up = new Vector3D(0, 0, 1);

                        // Calculate left direction perpendicular to stance-focal line
                        Vector3D leftDir = new Vector3D();
                        leftDir.cross(direction, up);

                        if (leftDir.norm() > 1e-5)
                        {
                           leftDir.normalize();
                           leftDir.scale(distanceToReferenceFrame);
                           goalStancePoint.add(leftDir);
                        }
                     }
                     case RIGHT ->
                     {
                        // Get world up vector (Z-axis)
                        Vector3D up = new Vector3D(0, 0, 1);

                        // Calculate right direction perpendicular to stance-focal line
                        Vector3D rightDir = new Vector3D();
                        rightDir.cross(up, direction);

                        if (rightDir.norm() > 1e-5)
                        {
                           rightDir.normalize();
                           rightDir.scale(distanceToReferenceFrame);
                           goalStancePoint.add(rightDir);
                        }
                     }
                  }
               }
               // After calculation, go back to pointing at object
               goalFocalPoint.changeFrame(referenceFrame);
               goalFocalPoint.setToZero();
               goalStancePoint.changeFrame(referenceFrame);
               gotoActionState.getDefinition().getGoalStancePoint().getValue().set(goalStancePoint);
               gotoActionState.getDefinition().getGoalFocalPoint().getValue().set(goalFocalPoint);
               break;
            }
         }
      }
   }

   private void updateReceiveObject(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message)
   {
      if (behaviorToExecuteName.contains("RECEIVE") && message.getAdaptingBehavior())
      {
         AI2RReceiveObjectMessage receiveMessage = message.getReceiveObject();
         String receiveObject = receiveMessage.getObjectNameAsString();
         if (!receiveObject.isEmpty())
         {
            objectGrasped = receiveObject;
         }
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf instanceof ConditionNodeState conditionNodeState)
            {
               if (conditionNodeState.getParent().getDefinition().getName().contains("ReceiveObject"))
               {
                  conditionNodeState.getDefinition().getProximityCheck().setFrameNameA(objectGrasped);
                  conditionNodeState.getDefinition().getProximityCheck().setFrameNameB(RobotSide.fromByte(receiveMessage.getSide()) == RobotSide.LEFT ? "leftHandZUp" : "rightHandZUp");
               }
            }
         }
      }
   }

   private void updatePickUpObject(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message)
   {
      if (behaviorToExecuteName.contains("PICK") && message.getAdaptingBehavior())
      {
         AI2RPickUpObjectMessage pickUpMessage = message.getPickupObject();
         String targetObject = pickUpMessage.getObjectNameAsString();
         if (!targetObject.isEmpty())
         {
            objectGrasped = targetObject;
         }
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf instanceof HandPoseActionState handPoseActionStateNodeState)
            {
               if (handPoseActionStateNodeState.getDefinition().getName().contains("Grasp"))
               {
                  handPoseActionStateNodeState.getDefinition().setPalmParentFrameName(targetObject);
               }
            }
         }
      }
   }

   private void changeParentFrameGoToNode(FootstepPlanActionDefinition definition, FootstepPlanActionState state, String newParentFrameName)
   {
      definition.setParentFrameName(newParentFrameName);
      // Timestamp modification to prevent the frame from glitching when changing frames
      state.getGoalFrame().changeFrame(newParentFrameName, state.getGoalToParentTransform().getValueAndModify());

      ReferenceFrame newParent = state.getGoalFrame().getReferenceFrame().getParent();
      FramePoint3D frameStancePoint;
      FramePoint3D frameFocalPoint;
      if (newParent.getRootFrame() == state.getParentFrame().getRootFrame())
      {
         // Keep the points in the same place w.r.t common ancestor frames
         frameStancePoint = new FramePoint3D(state.getParentFrame(), definition.getGoalStancePoint().getValueReadOnly());
         frameFocalPoint = new FramePoint3D(state.getParentFrame(), definition.getGoalFocalPoint().getValueReadOnly());
         frameStancePoint.changeFrame(newParent);
         frameFocalPoint.changeFrame(newParent);
      }
      else // The frame was detached so we keep the same transform to the new frame
      {
         frameStancePoint = new FramePoint3D(newParent, definition.getGoalStancePoint().getValueReadOnly());
         frameFocalPoint = new FramePoint3D(newParent, definition.getGoalFocalPoint().getValueReadOnly());
      }

      definition.getGoalStancePoint().getValueAndModify().set(frameStancePoint);
      definition.getGoalFocalPoint().getValueAndModify().set(frameFocalPoint);

      for (FootstepPlanActionFootstepState footstepState : state.getManuallyPlacedFootsteps())
      {
         footstepState.getSoleFrame().changeFrame(newParentFrameName);
      }
   }

//   private void updatePickAndPlace(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message)
//   {
//      if (behaviorToExecuteName.contains("PICKUP") || behaviorToExecuteName.contains("PLACE") && message.getAdaptingBehavior())
//      {
//         AI2RHandPoseAdaptationMessage handMessage = message.getHandPoseAdaptation();
//         for (var leaf : state.getActionSequence().getOrderedLeaves())
//         {
//            if (leaf.getLeafIndex() > commandedBehaviorIndex &&
//                leaf.getDefinition().getName().contains(handMessage.getActionName()) &&
//                leaf instanceof HandPoseActionState handPoseActionState)
//            {
//               handPoseActionState.getDefinition().setPalmParentFrameName(handMessage.getReferenceFrameNameAsString());
//               RigidBodyTransform adaptedPose = new RigidBodyTransform(handMessage.getNewOrientation(), handMessage.getNewPosition());
//               handPoseActionState.getDefinition().getPalmTransformToParent().setValue(adaptedPose ,1e-5);
//               break;
//            }
//         }
//      }
//   }

   public String getObjectGrasped()
   {
      return objectGrasped;
   }

   public RobotSide getGraspSide()
   {
      return graspSide;
   }

   public int getCommandedBehaviorIndex()
   {
      return commandedBehaviorIndex;
   }
}