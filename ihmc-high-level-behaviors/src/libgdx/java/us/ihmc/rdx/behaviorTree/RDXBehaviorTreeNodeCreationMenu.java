package us.ihmc.rdx.behaviorTree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.control.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.behaviorTree.control.door.DoorTraversalDefinition;
import us.ihmc.behaviors.behaviorTree.control.CheckpointNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceDefinition;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.*;
import us.ihmc.behaviors.behaviorTree.action.actions.PelvisActionDefinition;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.behaviorTree.actions.RDXActionNode;
import us.ihmc.rdx.ui.widgets.ImGuiArmIconWidget;
import us.ihmc.rdx.ui.widgets.ImGuiCheckpointNodeWidget;
import us.ihmc.rdx.ui.widgets.ImGuiConditionNodeWidget;
import us.ihmc.rdx.ui.widgets.ImGuiDoorNodeWidget;
import us.ihmc.rdx.ui.widgets.ImGuiFallbackWidget;
import us.ihmc.rdx.ui.widgets.ImGuiFootstepsWidget;
import us.ihmc.rdx.ui.widgets.ImGuiGotoNodeWidget;
import us.ihmc.rdx.ui.widgets.ImGuiGripperWidget;
import us.ihmc.rdx.ui.widgets.ImGuiHandWidget;
import us.ihmc.rdx.ui.widgets.ImGuiSceneActionWidget;
import us.ihmc.rdx.ui.widgets.ImGuiSequenceIconWidget;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class RDXBehaviorTreeNodeCreationMenu
{
   private final RDXBehaviorTree behaviorTree;
   private final BehaviorTreeTopologyOperationQueue<RDXBehaviorTreeNode<?, ?>> topologyOperationQueue;
   private final RDXAvailableBehaviorTreeDirectory behaviorTreesDirectory;
   private final ImGuiSequenceIconWidget sequenceIcon = new ImGuiSequenceIconWidget();
   private final ImGuiFallbackWidget fallbackIcon = new ImGuiFallbackWidget();
   private final ImGuiConditionNodeWidget conditionIcon = new ImGuiConditionNodeWidget();
   private final ImGuiGotoNodeWidget gotoIcon = new ImGuiGotoNodeWidget();
   private final ImGuiCheckpointNodeWidget checkpointIcon = new ImGuiCheckpointNodeWidget();
   private final ImGuiSceneActionWidget sceneActionIcon = new ImGuiSceneActionWidget();
   private final ImGuiDoorNodeWidget doorIcon = new ImGuiDoorNodeWidget();
   private final ImGuiFootstepsWidget footstepsIcon = new ImGuiFootstepsWidget();
   private final ImGuiArmIconWidget armIcon = new ImGuiArmIconWidget();
   private final ImGuiHandWidget handIcon = new ImGuiHandWidget();
   private final ImGuiGripperWidget gripperIcon = new ImGuiGripperWidget();

   public RDXBehaviorTreeNodeCreationMenu(RDXBehaviorTree behaviorTree, WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.behaviorTree = behaviorTree;

      topologyOperationQueue = behaviorTree.getTopologyChangeQueue();

      behaviorTreesDirectory = new RDXAvailableBehaviorTreeDirectory(treeFilesDirectory, behaviorTree, topologyOperationQueue, this::complete);
      behaviorTreesDirectory.reindexDirectory();
   }

   /**
    * This method assumes that the insertion is valid for the relative node.
    * For example, if the insertion requires modifying a parent, we assume it is not null.
    */
   public void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> relativeNode, BehaviorTreeNodeInsertionType insertionType)
   {
      if (relativeNode == null)
      {
         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         ImGui.text("Start from scratch:");
         ImGui.popFont();
         ImGui.indent();

         renderNodeCreationClickable(relativeNode, insertionType, "Root Node", BehaviorTreeRootNodeDefinition.class, null);
      }
      else
      {
         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         ImGui.text("Control nodes:");
         ImGui.popFont();
         ImGui.indent();

         float align = ImGui.getCursorPosX() + ImGui.getFontSize() * 2.0f;
         sequenceIcon.render(ImGui.getTextLineHeight());
         ImGui.sameLine();
         renderNodeCreationClickable(relativeNode, insertionType, "Action Sequence", ActionSequenceDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * -0.2f);
         fallbackIcon.render(ImGui.getTextLineHeight());
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Fallback Node", FallbackNodeDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         conditionIcon.render(ImGui.getTextLineHeight());
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Condition Node", ConditionNodeDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         gotoIcon.render();
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Goto Node", GotoNodeDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         checkpointIcon.render();
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Checkpoint Node", CheckpointNodeDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         sceneActionIcon.render();
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Scene Action", SceneActionDefinition.class, null);
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "AI2R Node", AI2RNodeDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         doorIcon.render();
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Door Traversal", DoorTraversalDefinition.class, null);
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Building Exploration", BuildingExplorationDefinition.class, null);
         ImGui.setCursorPosX(align);
         ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));
         renderNodeCreationClickable(relativeNode, insertionType, "Basic Node", BehaviorTreeNodeDefinition.class, null);
         ImGui.popStyleColor();

         ImGui.unindent();

         ImGui.separator();

         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         ImGui.text("Actions:");
         ImGui.popFont();
         ImGui.indent();

         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Wait", WaitActionDefinition.class, null);
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Mimic", MimicActionDefinition.class, null);
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Neck", NeckActionDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         armIcon.render(RobotSide.LEFT, false, false);
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         for (RobotSide side : RobotSide.values)
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), ArmActionDefinition.class, side);
         ImGui.text("Arm");
         ImGui.setCursorPosX(align);
         for (RobotSide side : RobotSide.values)
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), ScrewPrimitiveActionDefinition.class, side);
         ImGui.text("Screw Primitive");
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         handIcon.render(RobotSide.LEFT, ImGui.getTextLineHeight(), false);
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         for (RobotSide side : RobotSide.values)
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), AbilityHandActionDefinition.class, side);
         ImGui.text("Ability Hand");
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         gripperIcon.render(RobotSide.LEFT, ImGui.getTextLineHeight());
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         for (RobotSide side : RobotSide.values)
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), EZGripperActionDefinition.class, side);
         ImGui.text("EZGripper");
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Spine", SpineActionDefinition.class, null);
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Pelvis", PelvisActionDefinition.class, null);
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() * 0.3f);
         footstepsIcon.render(ImGui.getTextLineHeight());
         ImGui.sameLine();
         ImGui.setCursorPosX(align);
         renderNodeCreationClickable(relativeNode, insertionType, "Walk", WalkActionDefinition.class, null);
         ImGui.setCursorPosX(align);
         for (RobotSide side : RobotSide.values)
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), LegActionDefinition.class, side);
         ImGui.text("Leg");
         ImGui.setCursorPosX(align);
         for (RobotSide side : RobotSide.values)
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), HandWrenchActionDefinition.class, side);
         ImGui.textDisabled("Hand Wrench");
      }
      ImGui.unindent();
      ImGui.spacing();
      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("Load existing tree from file:");
      ImGui.popFont();

      ImGui.unindent();
      behaviorTreesDirectory.renderImGuiWidgets(relativeNode, insertionType, true);
      ImGui.indent();
   }

   private void renderNodeCreationClickable(RDXBehaviorTreeNode<?, ?> relativeNode,
                                            BehaviorTreeNodeInsertionType insertionType,
                                            String nodeTypeName,
                                            Class<?> nodeType,
                                            @Nullable RobotSide side)
   {
      if (ImGuiTools.textWithUnderlineOnHover(nodeTypeName))
      {
         if (ImGui.isMouseClicked(ImGuiMouseButton.Left))
         {
            RDXBehaviorTreeNode<?, ?> newNode;
            if (insertionType == BehaviorTreeNodeInsertionType.INSERT_ROOT)
               newNode = (RDXBehaviorTreeNode<?, ?>) behaviorTree.getNodeBuilder().createRootNode(behaviorTree.getAndIncrementNextID());
            else
               newNode = behaviorTree.getNodeBuilder().createNode(nodeType, behaviorTree.getAndIncrementNextID(), behaviorTree.getRootNode());
            newNode.getDefinition().modify();
            BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                  = new BehaviorTreeNodeInsertionDefinition<>(insertionType, newNode, relativeNode);

            if (insertionDefinition.getNodeToInsert() instanceof RDXActionNode<?, ?> newAction)
            {
               // We want to do best effort initialization
               RDXBehaviorTreeRootNode actionSequenceOrNull = behaviorTree.getRootNode();
               if (behaviorTree.getNodeBuilder() instanceof RDXBehaviorTreeNodeBuilder rdxNodeBuilder)
                  rdxNodeBuilder.initializeActionNode(actionSequenceOrNull, newAction, insertionDefinition.getInsertionIndex(), side);
            }

            complete(insertionDefinition);
         }
      }
      if (side != null)
         ImGui.sameLine();
   }

   private void complete(BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition)
   {
      topologyOperationQueue.queueInsertNodeModify(insertionDefinition);
      ImGui.closeCurrentPopup();
   }

   public void reindexDirectory()
   {
      behaviorTreesDirectory.reindexDirectory();
   }
}
