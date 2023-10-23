package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateBuilder;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
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
   private final FootstepPlannerParametersBasics footstepPlannerParametersBasics;
   private final ROS2ControllerPublishSubscribeAPI ros2;

   public RDXBehaviorTreeNodeBuilder(DRCRobotModel robotModel,
                                     ROS2SyncedRobotModel syncedRobot,
                                     RobotCollisionModel selectionCollisionModel,
                                     RDXBaseUI baseUI,
                                     RDX3DPanel panel3D,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     FootstepPlannerParametersBasics footstepPlannerParametersBasics,
                                     ROS2ControllerPublishSubscribeAPI ros2)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.baseUI = baseUI;
      this.panel3D = panel3D;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.footstepPlannerParametersBasics = footstepPlannerParametersBasics;
      this.ros2 = ros2;
   }

   @Override
   public RDXBehaviorTreeNode createNode(Class<?> nodeType, long id)
   {
      RDXBehaviorActionSequenceEditor editor = null; // TODO ???? Probably need the parent sequence node?

      if (nodeType == ArmJointAnglesActionDefinition.class)
      {
         return new RDXArmJointAnglesAction(id, editor, robotModel);
      }
      if (nodeType == ChestOrientationActionDefinition.class)
      {
         return new RDXChestOrientationAction(id,
                                              editor,
                                              panel3D,
                                              robotModel,
                                              syncedRobot.getFullRobotModel(),
                                              selectionCollisionModel,
                                              referenceFrameLibrary,
                                              ros2);
      }
      if (nodeType == FootstepPlanActionDefinition.class)
      {
         return new RDXFootstepPlanAction(id, editor, baseUI, robotModel, syncedRobot, referenceFrameLibrary);
      }
      if (nodeType == HandPoseActionDefinition.class)
      {
         return new RDXHandPoseAction(id, editor, panel3D, robotModel, syncedRobot.getFullRobotModel(), selectionCollisionModel, referenceFrameLibrary, ros2);
      }
      if (nodeType == HandWrenchActionDefinition.class)
      {
         return new RDXHandWrenchAction(id, editor);
      }
      if (nodeType == PelvisHeightPitchActionDefinition.class)
      {
         return new RDXPelvisHeightPitchAction(id,
                                               editor,
                                               panel3D,
                                               robotModel,
                                               syncedRobot.getFullRobotModel(),
                                               selectionCollisionModel,
                                               referenceFrameLibrary,
                                               ros2);
      }
      if (nodeType == SakeHandCommandActionDefinition.class)
      {
         return new RDXSakeHandCommandAction(id, editor);
      }
      if (nodeType == WaitDurationActionDefinition.class)
      {
         return new RDXWaitDurationAction(id, editor);
      }
      if (nodeType == WalkActionDefinition.class)
      {
         return new RDXWalkAction(id, editor, panel3D, robotModel, referenceFrameLibrary, footstepPlannerParametersBasics);
      }
      else
      {
         return null;
      }
   }
}
