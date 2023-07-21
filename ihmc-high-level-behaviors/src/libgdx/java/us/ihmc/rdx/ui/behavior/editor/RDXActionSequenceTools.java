package us.ihmc.rdx.ui.behavior.editor;

import behavior_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.editor.actions.*;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.ArrayList;
import java.util.List;

public class RDXActionSequenceTools
{
   public static RDXBehaviorAction createBlankAction(String actionType,
                                                     DRCRobotModel robotModel,
                                                     ROS2SyncedRobotModel syncedRobot,
                                                     RobotCollisionModel selectionCollisionModel,
                                                     RDX3DPanel panel3D,
                                                     ReferenceFrameLibrary referenceFrameLibrary,
                                                     ROS2ControllerPublishSubscribeAPI ros2)
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
            return robotHasArms ?
                  new RDXHandPoseAction(panel3D, robotModel, syncedRobot.getFullRobotModel(), selectionCollisionModel, referenceFrameLibrary, ros2) :
                  null;
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

   public static void packActionSequenceUpdateMessage(List<RDXBehaviorAction> actionSequence,
                                                      ArrayList<BehaviorActionData> actionDataForMessage,
                                                      ActionSequenceUpdateMessage actionSequenceUpdateMessage)
   {
      actionDataForMessage.clear();
      for (RDXBehaviorAction behaviorAction : actionSequence)
      {
         actionDataForMessage.add(behaviorAction.getActionData());
      }
      BehaviorActionSequenceTools.packActionSequenceUpdateMessage(actionDataForMessage, actionSequenceUpdateMessage);
   }
}
