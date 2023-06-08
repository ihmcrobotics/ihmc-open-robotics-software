package us.ihmc.rdx.ui.behavior.editor;

import behavior_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.editor.actions.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.List;
import java.util.UUID;

public class RDXActionSequenceTools
{
   public static RDXBehaviorAction createBlankAction(String actionType,
                                                     DRCRobotModel robotModel,
                                                     ROS2SyncedRobotModel syncedRobot,
                                                     RDX3DPanel panel3D,
                                                     ReferenceFrameLibrary referenceFrameLibrary)
   {
      boolean robotHasArms = robotModel.getRobotVersion().hasArms();
      switch (actionType)
      {
         case "RDXArmJointAnglesAction" ->
         {
            return robotHasArms ? new RDXArmJointAnglesAction() : null;
         }
         case "RDXChestOrientationAction" ->
         {
            return new RDXChestOrientationAction();
         }
         case "RDXFootstepAction" ->
         {
            return new RDXFootstepAction(panel3D, robotModel, syncedRobot, referenceFrameLibrary);
         }
         case "RDXHandConfigurationAction" ->
         {
            return robotHasArms ? new RDXHandConfigurationAction() : null;
         }
         case "RDXHandPoseAction" ->
         {
            return robotHasArms ? new RDXHandPoseAction(panel3D, robotModel, syncedRobot.getFullRobotModel(), referenceFrameLibrary) : null;
         }
         case "RDXHandWrenchAction" ->
         {
            return robotHasArms ? new RDXHandWrenchAction() : null;
         }
         case "RDXPelvisHeightAction" ->
         {
            return new RDXPelvisHeightAction();
         }
         case "RDXWaitDurationAction" ->
         {
            return new RDXWaitDurationAction();
         }
         case "RDXWalkAction" ->
         {
            return new RDXWalkAction(panel3D, robotModel, referenceFrameLibrary);
         }
      };

      return null;
   }

   public static void publishActionSequenceUpdateMessage(List<RDXBehaviorAction> actionSequence, ROS2PublishSubscribeAPI ros2)
   {
      long updateUUID = UUID.randomUUID().getLeastSignificantBits();
      ActionSequenceUpdateMessage actionSequenceUpdateMessage = new ActionSequenceUpdateMessage();
      actionSequenceUpdateMessage.setSequenceUpdateUuid(updateUUID);
      actionSequenceUpdateMessage.setSequenceSize(actionSequence.size());
      ros2.publish(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC, actionSequenceUpdateMessage);

      for (int i = 0; i < actionSequence.size(); i++)
      {
         RDXBehaviorAction action = actionSequence.get(i);
         if (action instanceof RDXArmJointAnglesAction armJointAnglesAction)
         {
            ArmJointAnglesActionMessage armJointAnglesActionMessage = new ArmJointAnglesActionMessage();
            armJointAnglesActionMessage.getActionInformation().setActionIndex(i);
            armJointAnglesActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            armJointAnglesAction.getActionData().toMessage(armJointAnglesActionMessage);
            ros2.publish(BehaviorActionSequence.ARM_JOINT_ANGLES_UPDATE_TOPIC, armJointAnglesActionMessage);
         }
         else if (action instanceof RDXChestOrientationAction chestOrientationAction)
         {
            ChestOrientationActionMessage chestOrientationActionMessage = new ChestOrientationActionMessage();
            chestOrientationActionMessage.getActionInformation().setActionIndex(i);
            chestOrientationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            chestOrientationAction.getActionData().toMessage(chestOrientationActionMessage);
            ros2.publish(BehaviorActionSequence.CHEST_ORIENTATION_UPDATE_TOPIC, chestOrientationActionMessage);
         }
         else if (action instanceof RDXFootstepAction footstepAction)
         {
            FootstepActionMessage footstepActionMessage = new FootstepActionMessage();
            footstepActionMessage.getActionInformation().setActionIndex(i);
            footstepActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            footstepAction.getActionData().toMessage(footstepActionMessage);
            ros2.publish(BehaviorActionSequence.FOOTSTEP_UPDATE_TOPIC, footstepActionMessage);
         }
         else if (action instanceof RDXHandConfigurationAction handConfigurationAction)
         {
            HandConfigurationActionMessage handConfigurationActionMessage = new HandConfigurationActionMessage();
            handConfigurationActionMessage.getActionInformation().setActionIndex(i);
            handConfigurationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            handConfigurationAction.getActionData().toMessage(handConfigurationActionMessage);
            ros2.publish(BehaviorActionSequence.HAND_CONFIGURATION_UPDATE_TOPIC, handConfigurationActionMessage);
         }
         else if (action instanceof RDXHandPoseAction handPoseAction)
         {
            HandPoseActionMessage handPoseActionMessage = new HandPoseActionMessage();
            handPoseActionMessage.getActionInformation().setActionIndex(i);
            handPoseActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            handPoseAction.getActionData().toMessage(handPoseActionMessage);
            ros2.publish(BehaviorActionSequence.HAND_POSE_UPDATE_TOPIC, handPoseActionMessage);
         }
         else if (action instanceof RDXHandWrenchAction handWrenchAction)
         {
            HandWrenchActionMessage handWrenchActionMessage = new HandWrenchActionMessage();
            handWrenchActionMessage.getActionInformation().setActionIndex(i);
            handWrenchActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            handWrenchAction.getActionData().toMessage(handWrenchActionMessage);
            ros2.publish(BehaviorActionSequence.HAND_WRENCH_UPDATE_TOPIC, handWrenchActionMessage);
         }
         else if (action instanceof RDXPelvisHeightAction pelvisHeightAction)
         {
            PelvisHeightActionMessage pelvisHeightActionMessage = new PelvisHeightActionMessage();
            pelvisHeightActionMessage.getActionInformation().setActionIndex(i);
            pelvisHeightActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            pelvisHeightAction.getActionData().toMessage(pelvisHeightActionMessage);
            ros2.publish(BehaviorActionSequence.PELVIS_HEIGHT_UPDATE_TOPIC, pelvisHeightActionMessage);
         }
         else if (action instanceof RDXWaitDurationAction waitDurationAction)
         {
            WaitDurationActionMessage waitDurationActionMessage = new WaitDurationActionMessage();
            waitDurationActionMessage.getActionInformation().setActionIndex(i);
            waitDurationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            waitDurationAction.getActionData().toMessage(waitDurationActionMessage);
            ros2.publish(BehaviorActionSequence.WAIT_DURATION_UPDATE_TOPIC, waitDurationActionMessage);
         }
         else if (action instanceof RDXWalkAction walkAction)
         {
            WalkActionMessage walkActionMessage = new WalkActionMessage();
            walkActionMessage.getActionInformation().setActionIndex(i);
            walkActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            walkAction.getActionData().toMessage(walkActionMessage);
            ros2.publish(BehaviorActionSequence.WALK_UPDATE_TOPIC, walkActionMessage);
         }
      }
   }
}
