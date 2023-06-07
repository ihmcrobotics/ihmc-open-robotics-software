package us.ihmc.rdx.ui.behavior.editor;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.editor.actions.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

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
}
