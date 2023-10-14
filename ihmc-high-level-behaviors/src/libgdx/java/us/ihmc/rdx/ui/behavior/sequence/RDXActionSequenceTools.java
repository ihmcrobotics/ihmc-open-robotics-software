package us.ihmc.rdx.ui.behavior.sequence;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.actions.*;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class RDXActionSequenceTools
{
   public static <T> T createBlankAction(Class<T> actionType,
                                         RDXBehaviorActionSequenceEditor editor,
                                         DRCRobotModel robotModel,
                                         ROS2SyncedRobotModel syncedRobot,
                                         RobotCollisionModel selectionCollisionModel,
                                         RDXBaseUI baseUI,
                                         RDX3DPanel panel3D,
                                         ReferenceFrameLibrary referenceFrameLibrary,
                                         ROS2ControllerPublishSubscribeAPI ros2)
   {
      return (T) createBlankAction(actionType.getSimpleName(),
                                   editor,
                                   robotModel,
                                   syncedRobot,
                                   selectionCollisionModel,
                                   baseUI,
                                   panel3D,
                                   referenceFrameLibrary,
                                   ros2);
   }

   public static RDXBehaviorAction createBlankAction(String actionType,
                                                     RDXBehaviorActionSequenceEditor editor,
                                                     DRCRobotModel robotModel,
                                                     ROS2SyncedRobotModel syncedRobot,
                                                     RobotCollisionModel selectionCollisionModel,
                                                     RDXBaseUI baseUI,
                                                     RDX3DPanel panel3D,
                                                     ReferenceFrameLibrary referenceFrameLibrary,
                                                     ROS2ControllerPublishSubscribeAPI ros2)
   {
      boolean robotHasArms = robotModel.getRobotVersion().hasArms();
      if (actionType.equals(ArmJointAnglesActionDefinition.class.getSimpleName()))
      {
         return robotHasArms ? new RDXArmJointAnglesAction(editor, robotModel) : null;
      }
      if (actionType.equals(ChestOrientationActionDefinition.class.getSimpleName()))
      {
         return new RDXChestOrientationAction(editor,
                                              panel3D,
                                              robotModel,
                                              syncedRobot.getFullRobotModel(),
                                              selectionCollisionModel,
                                              referenceFrameLibrary,
                                              ros2);
      }
      if (actionType.equals(FootstepPlanActionDefinition.class.getSimpleName()))
      {
         return new RDXFootstepPlanAction(editor, baseUI, robotModel, syncedRobot, referenceFrameLibrary);
      }
      if (actionType.equals(SakeHandCommandActionDefinition.class.getSimpleName()))
      {
         return robotHasArms ? new RDXSakeHandCommandAction(editor) : null;
      }
      if (actionType.equals(HandPoseActionDefinition.class.getSimpleName()))
      {
         return robotHasArms ?
               new RDXHandPoseAction(editor, panel3D, robotModel, syncedRobot.getFullRobotModel(), selectionCollisionModel, referenceFrameLibrary, ros2) :
               null;
      }
      if (actionType.equals(HandWrenchActionDefinition.class.getSimpleName()))
      {
         return robotHasArms ? new RDXHandWrenchAction(editor) : null;
      }
      if (actionType.equals(PelvisHeightPitchActionDefinition.class.getSimpleName()))
      {
         return new RDXPelvisHeightPitchAction(editor,
                                               panel3D,
                                               robotModel,
                                               syncedRobot.getFullRobotModel(),
                                               selectionCollisionModel,
                                               referenceFrameLibrary,
                                               ros2);
      }
      if (actionType.equals(WaitDurationActionDefinition.class.getSimpleName()))
      {
         return new RDXWaitDurationAction(editor);
      }
      if (actionType.equals(WalkActionDefinition.class.getSimpleName()))
      {
         return new RDXWalkAction(editor, panel3D, robotModel, referenceFrameLibrary);
      }

      return null;
   }
}
