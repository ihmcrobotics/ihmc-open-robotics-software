package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.actions.*;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorActionSequenceEditor;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class RDXBehaviorTreeTools
{
   public static Class<?> getClassFromTypeName(String typeName)
   {
      if (typeName.equals(ArmJointAnglesActionDefinition.class.getSimpleName()))
      {
         return RDXArmJointAnglesAction.class;
      }
      if (typeName.equals(ChestOrientationActionDefinition.class.getSimpleName()))
      {
         return RDXChestOrientationAction.class;
      }
      if (typeName.equals(FootstepPlanActionDefinition.class.getSimpleName()))
      {
         return RDXFootstepPlanAction.class;
      }
      if (typeName.equals(HandPoseActionDefinition.class.getSimpleName()))
      {
         return RDXHandPoseAction.class;
      }
      if (typeName.equals(HandWrenchActionDefinition.class.getSimpleName()))
      {
         return RDXHandWrenchAction.class;
      }
      if (typeName.equals(PelvisHeightPitchActionDefinition.class.getSimpleName()))
      {
         return RDXPelvisHeightPitchAction.class;
      }
      if (typeName.equals(SakeHandCommandActionDefinition.class.getSimpleName()))
      {
         return RDXSakeHandCommandAction.class;
      }
      if (typeName.equals(WaitDurationActionDefinition.class.getSimpleName()))
      {
         return RDXWaitDurationAction.class;
      }
      if (typeName.equals(WalkActionDefinition.class.getSimpleName()))
      {
         return RDXWalkAction.class;
      }
      else
      {
         return null;
      }
   }

   public static RDXBehaviorTreeNode createNode(Class<?> nodeType,
                                                long id,
                                                DRCRobotModel robotModel,
                                                ROS2SyncedRobotModel syncedRobot,
                                                RobotCollisionModel selectionCollisionModel,
                                                RDXBaseUI baseUI,
                                                RDX3DPanel panel3D,
                                                ReferenceFrameLibrary referenceFrameLibrary,
                                                ROS2ControllerPublishSubscribeAPI ros2)
   {

      RDXBehaviorActionSequenceEditor editor = null; // TODO ????

      if (nodeType == RDXArmJointAnglesAction.class)
      {
         return new RDXArmJointAnglesAction(editor, robotModel);
      }
      if (nodeType == RDXChestOrientationAction.class)
      {
         return new RDXChestOrientationAction(editor,
                                              panel3D,
                                              robotModel,
                                              syncedRobot.getFullRobotModel(),
                                              selectionCollisionModel,
                                              referenceFrameLibrary,
                                              ros2);
      }
      if (nodeType == RDXFootstepPlanAction.class)
      {
         return new RDXFootstepPlanAction(editor, baseUI, robotModel, syncedRobot, referenceFrameLibrary);
      }
      if (nodeType == RDXHandPoseAction.class)
      {
         return new RDXHandPoseAction(editor, panel3D, robotModel, syncedRobot.getFullRobotModel(), selectionCollisionModel, referenceFrameLibrary, ros2);
      }
      if (nodeType == RDXHandWrenchAction.class)
      {
         return new RDXHandWrenchAction(editor);
      }
      if (nodeType == RDXPelvisHeightPitchAction.class)
      {
         return new RDXPelvisHeightPitchAction(editor,
                                               panel3D,
                                               robotModel,
                                               syncedRobot.getFullRobotModel(),
                                               selectionCollisionModel,
                                               referenceFrameLibrary,
                                               ros2);
      }
      if (nodeType == RDXSakeHandCommandAction.class)
      {
         return new RDXSakeHandCommandAction(editor);
      }
      if (nodeType == RDXWaitDurationAction.class)
      {
         return new RDXWaitDurationAction(editor);
      }
      if (nodeType == RDXWalkAction.class)
      {
         return new RDXWalkAction(editor, panel3D, robotModel, referenceFrameLibrary);
      }
      else
      {
         return null;
      }
   }
}
