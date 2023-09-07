package us.ihmc.rdx.ui.behavior.editor;

import behavior_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
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
                                                     RDXBaseUI baseUI,
                                                     RDX3DPanel panel3D,
                                                     ReferenceFrameLibrary referenceFrameLibrary,
                                                     ROS2ControllerPublishSubscribeAPI ros2)
   {
      boolean robotHasArms = robotModel.getRobotVersion().hasArms();
      if (actionType.equals(RDXArmJointAnglesAction.class.getSimpleName()))
      {
         return robotHasArms ? new RDXArmJointAnglesAction(robotModel) : null;
      }
      if (actionType.equals(RDXChestOrientationAction.class.getSimpleName()))
      {
         return new RDXChestOrientationAction();
      }
      if (actionType.equals(RDXFootstepPlanAction.class.getSimpleName()))
      {
         return new RDXFootstepPlanAction(baseUI, robotModel, syncedRobot, referenceFrameLibrary);
      }
      if (actionType.equals(RDXHandConfigurationAction.class.getSimpleName()))
      {
         return robotHasArms ? new RDXHandConfigurationAction() : null;
      }
      if (actionType.equals(RDXHandPoseAction.class.getSimpleName()))
      {
         return robotHasArms ?
               new RDXHandPoseAction(panel3D, robotModel, syncedRobot.getFullRobotModel(), selectionCollisionModel, referenceFrameLibrary, ros2)
               : null;
      }
      if (actionType.equals(RDXHandWrenchAction.class.getSimpleName()))
      {
         return robotHasArms ? new RDXHandWrenchAction() : null;
      }
      if (actionType.equals(RDXPelvisHeightAction.class.getSimpleName()))
      {
         return new RDXPelvisHeightAction(panel3D, robotModel, syncedRobot.getFullRobotModel(), selectionCollisionModel, referenceFrameLibrary);
      }
      if (actionType.equals(RDXWaitDurationAction.class.getSimpleName()))
      {
         return new RDXWaitDurationAction();
      }
      if (actionType.equals(RDXWalkAction.class.getSimpleName()))
      {
         return new RDXWalkAction(panel3D, robotModel, referenceFrameLibrary);
      }

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
