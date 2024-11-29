package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.behaviors.sequence.actions.PelvisHeightOrientationActionDefinition;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeDirectory;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class RDXBehaviorTreeNodeCreationMenu
{
   private final RDXBehaviorTree tree;
   private final BehaviorTreeTopologyOperationQueue topologyOperationQueue;
   private final RDXAvailableBehaviorTreeDirectory behaviorTreesDirectory;

   public RDXBehaviorTreeNodeCreationMenu(RDXBehaviorTree tree, WorkspaceResourceDirectory treeFilesDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.tree = tree;

      topologyOperationQueue = tree.getBehaviorTreeState().getTopologyChangeQueue();

      behaviorTreesDirectory = new RDXAvailableBehaviorTreeDirectory(treeFilesDirectory, tree, topologyOperationQueue, referenceFrameLibrary, this::complete);
      behaviorTreesDirectory.reindexDirectory();
   }

   /**
    * This method assumes that the insertion is valid for the relative node.
    * For example, if the insertion requires modifying a parent, we assume it is not null.
    */
   public void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> relativeNode, BehaviorTreeNodeInsertionType insertionType)
   {
      if (insertionType == BehaviorTreeNodeInsertionType.INSERT_ROOT)
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

         renderNodeCreationClickable(relativeNode, insertionType, "Basic Node", BehaviorTreeNodeDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "AI2R Node", AI2RNodeDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Action Sequence", ActionSequenceDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Door Traversal", DoorTraversalDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Trash Can Interaction", TrashCanInteractionDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Building Exploration", BuildingExplorationDefinition.class, null);

         ImGui.unindent();

         ImGui.separator();

         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         ImGui.text("Actions:");
         ImGui.popFont();
         ImGui.indent();

         renderNodeCreationClickable(relativeNode, insertionType, "Footstep Plan", FootstepPlanActionDefinition.class, null);
         ImGui.text("Foot Pose: ");
         for (RobotSide side : RobotSide.values)
         {
            ImGui.sameLine();
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), FootPoseActionDefinition.class, side);
         }
         ImGui.text("Hand Pose: ");
         for (RobotSide side : RobotSide.values)
         {
            ImGui.sameLine();
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), HandPoseActionDefinition.class, side);
         }
         ImGui.text("Sake Hand Command: ");
         for (RobotSide side : RobotSide.values)
         {
            ImGui.sameLine();
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), SakeHandCommandActionDefinition.class, side);
         }
         renderNodeCreationClickable(relativeNode, insertionType, "Chest Orientation", ChestOrientationActionDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Pelvis Height", PelvisHeightOrientationActionDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Wait", WaitDurationActionDefinition.class, null);
         ImGui.text("Screw Primitive: ");
         for (RobotSide side : RobotSide.values)
         {
            ImGui.sameLine();
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), ScrewPrimitiveActionDefinition.class, side);
         }
         ImGui.textDisabled("Hand Wrench: ");
         for (RobotSide side : RobotSide.values)
         {
            ImGui.sameLine();
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), HandWrenchActionDefinition.class, side);
         }
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
            RDXBehaviorTreeNode<?, ?> newNode = tree.getNodeBuilder()
                                                    .createNode(nodeType,
                                                                tree.getBehaviorTreeState().getAndIncrementNextID(),
                                                                tree.getBehaviorTreeState().getCRDTInfo(),
                                                                tree.getBehaviorTreeState().getSaveFileDirectory());

            BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                  = BehaviorTreeNodeInsertionDefinition.build(newNode, tree.getBehaviorTreeState(), tree::setRootNode, relativeNode, insertionType);

            if (insertionDefinition.getNodeToInsert() instanceof RDXActionNode<?, ?> newAction)
            {
               // We want to do best effort initialization
               RDXBehaviorTreeRootNode actionSequenceOrNull = tree.getRootNode();
               tree.getNodeBuilder().initializeActionNode(actionSequenceOrNull, newAction, insertionDefinition.getInsertionIndex(), side);
            }

            complete(insertionDefinition);
         }
      }
   }

   private void complete(BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition)
   {
      topologyOperationQueue.queueInsertNode(insertionDefinition);
      ImGui.closeCurrentPopup();

      if (insertionDefinition.getParent() != null)
         insertionDefinition.getParent().setTreeWidgetExpanded(true);

      insertionDefinition.getNodeToInsert().setTreeWidgetExpanded(true);
   }

   public void reindexDirectory()
   {
      behaviorTreesDirectory.reindexDirectory();
   }
}
