package us.ihmc.behaviors.ai2r;

import behavior_msgs.msg.dds.AI2RCommandMessage;
import behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage;
import behavior_msgs.msg.dds.AI2RNavigationMessage;
import behavior_msgs.msg.dds.AI2RReceiveObjectMessage;
import behavior_msgs.msg.dds.AI2RScanMessage;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDefinition;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionFootstepState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence.StringBuilderHolder;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

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
   private StringBuilderHolder objectsToScan;
   private String objectToTrack;
   private int commandedBehaviorIndex;

   public void adaptSkills(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message, int commandedBehaviorIndex)
   {
      this.commandedBehaviorIndex = commandedBehaviorIndex;
      updateScan(behaviorToExecuteName, state, message);
      updateGoTo(behaviorToExecuteName, state, message);
      updateReceiveObject(behaviorToExecuteName, state, message);
      updatePickAndPlace(behaviorToExecuteName, state, message);
   }

   private void updateScan(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message)
   {
      if (behaviorToExecuteName.contains("SCAN") && message.getAdaptingBehavior())
      {
         AI2RScanMessage scanMessage = message.getScan();
         objectsToScan = scanMessage.getTargetObjects();
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf instanceof ConditionNodeState conditionNodeState && conditionNodeState.getParent().getDefinition().getName().contains("Scan"))
            {
               String scanTarget = objectsToScan.get(0).toString();
               if (!scanTarget.isEmpty())
               {
                  objectToTrack = scanTarget;
               }
               conditionNodeState.getDefinition().getProximityCheck().setObjectFrameName(scanTarget);
            }
         }
      }
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
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf instanceof ConditionNodeState conditionNodeState && conditionNodeState.getParent().getDefinition().getName().contains("ReceiveObject"))
            {
//               AI2RReceiveObjectMessage receiveMessage = message.getReceiveObject();
//               String receiveObject = receiveMessage.getObjectNameAsString();
//               if (!receiveObject.isEmpty())
//               {
//                  objectToTrack = receiveObject;
//               }
               objectToTrack = "Charge";
               conditionNodeState.getDefinition().getProximityCheck().setObjectFrameName(objectToTrack);
               conditionNodeState.getDefinition().getProximityCheck().setReferenceFrameName("rightHandZUp");
//               conditionNodeState.getDefinition().getProximityCheck().setObjectFrameName(receiveObject);
//               conditionNodeState.getDefinition().getProximityCheck().setReferenceFrameName(RobotSide.fromByte(receiveMessage.getSide())==RobotSide.LEFT ? "leftHandZUp" : "rightHandZUp");
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

   private void updatePickAndPlace(String behaviorToExecuteName, AI2RNodeState state, AI2RCommandMessage message)
   {
      if (behaviorToExecuteName.contains("PICKUP") || behaviorToExecuteName.contains("PLACE") && message.getAdaptingBehavior())
      {
         AI2RHandPoseAdaptationMessage handMessage = message.getHandPoseAdaptation();
         for (var leaf : state.getActionSequence().getOrderedLeaves())
         {
            if (leaf.getLeafIndex() > commandedBehaviorIndex &&
                leaf.getDefinition().getName().contains(handMessage.getActionName()) &&
                leaf instanceof HandPoseActionState handPoseActionState)
            {
               handPoseActionState.getDefinition().setPalmParentFrameName(handMessage.getReferenceFrameNameAsString());
               RigidBodyTransform adaptedPose = new RigidBodyTransform(handMessage.getNewOrientation(), handMessage.getNewPosition());
               handPoseActionState.getDefinition().getPalmTransformToParent().setValue(adaptedPose ,1e-5);
               break;
            }
         }
      }
   }

   public String getObjectToTrack()
   {
      return objectToTrack;
   }

   public StringBuilderHolder getObjectsToScan()
   {
      return objectsToScan;
   }

   public int getCommandedBehaviorIndex()
   {
      return commandedBehaviorIndex;
   }
}
