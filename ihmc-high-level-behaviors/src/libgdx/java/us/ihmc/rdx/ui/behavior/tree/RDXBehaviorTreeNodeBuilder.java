package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateBuilder;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.actions.*;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorActionSequenceEditor;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class RDXBehaviorTreeNodeBuilder implements BehaviorTreeNodeStateBuilder
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RobotCollisionModel selectionCollisionModel;
   private final RDXBaseUI baseUI;
   private final RDX3DPanel panel3D;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2ControllerPublishSubscribeAPI ros2;

   public RDXBehaviorTreeNodeBuilder(DRCRobotModel robotModel,
                                     ROS2SyncedRobotModel syncedRobot,
                                     RobotCollisionModel selectionCollisionModel,
                                     RDXBaseUI baseUI,
                                     RDX3DPanel panel3D,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     ROS2ControllerPublishSubscribeAPI ros2)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.baseUI = baseUI;
      this.panel3D = panel3D;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.ros2 = ros2;
   }

   @Override
   public RDXBehaviorTreeNode createNode(Class<?> nodeType, long id)
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
