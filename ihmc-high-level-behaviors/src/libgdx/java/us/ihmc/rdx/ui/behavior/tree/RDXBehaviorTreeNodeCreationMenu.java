package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeFile;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Comparator;

public class RDXBehaviorTreeNodeCreationMenu
{
   private final RDXBehaviorTree tree;
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final BehaviorTreeTopologyOperationQueue topologyOperationQueue;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<RDXAvailableBehaviorTreeFile> indexedTreeFiles = new ArrayList<>();

   public RDXBehaviorTreeNodeCreationMenu(RDXBehaviorTree tree, WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.tree = tree;
      this.treeFilesDirectory = treeFilesDirectory;

      topologyOperationQueue = tree.getBehaviorTreeState().getTopologyChangeQueue();

      reindexDirectory();
   }

   /**
    * This method assumes that the insertion is valid for the relative node.
    * For example, if the insertion requires modifying a parent, we assume it is not null.
    */
   public void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> relativeNode, BehaviorTreeNodeInsertionType insertionType)
   {
      boolean parentIsActionSequenceNode = relativeNode instanceof RDXActionSequence && insertionType == BehaviorTreeNodeInsertionType.INSERT_AS_CHILD;
      boolean parentIsBasicNode = relativeNode != null
                               && relativeNode.getClass().equals(RDXBehaviorTreeNode.class)
                               && insertionType == BehaviorTreeNodeInsertionType.INSERT_AS_CHILD
                               && RDXBehaviorTreeTools.findActionSequenceAncestor(relativeNode) != null;
      boolean ancestorIsActionSequenceNode = relativeNode instanceof RDXActionNode<?, ?>
                                          && RDXBehaviorTreeTools.findActionSequenceAncestor(relativeNode) != null
            && (insertionType == BehaviorTreeNodeInsertionType.INSERT_AFTER || insertionType == BehaviorTreeNodeInsertionType.INSERT_BEFORE);

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("From file:");
      ImGui.popFont();

      ImGui.indent();
      for (RDXAvailableBehaviorTreeFile indexedTreeFile : indexedTreeFiles)
      {
         String fileName = indexedTreeFile.getTreeFile().getFileName();
         if (ImGuiTools.textWithUnderlineOnHover(fileName))
         {
            if (ImGui.isMouseClicked(ImGuiMouseButton.Left))
            {
               RDXBehaviorTreeNode<?, ?> loadedNode = tree.getFileLoader().loadFromFile(indexedTreeFile, topologyOperationQueue);

               BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                   = BehaviorTreeNodeInsertionDefinition.build(loadedNode, tree.getBehaviorTreeState(), tree::setRootNode, relativeNode, insertionType);

               complete(insertionDefinition);
            }
         }
      }
      ImGui.unindent();
      ImGui.spacing();

      if (ImGui.button(labels.get("Refresh File List")))
      {
         reindexDirectory();
      }

      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("Control nodes:");
      ImGui.popFont();
      ImGui.indent();

      if (relativeNode != null)
         renderNodeCreationClickable(relativeNode, insertionType, "Basic Node", BehaviorTreeNodeDefinition.class, null);
      if (insertionType == BehaviorTreeNodeInsertionType.INSERT_ROOT)
         renderNodeCreationClickable(relativeNode, insertionType, "Action Sequence", ActionSequenceDefinition.class, null);

      ImGui.unindent();

      if (parentIsActionSequenceNode || parentIsBasicNode || ancestorIsActionSequenceNode)
      {
         ImGui.separator();

         ImGui.pushFont(ImGuiTools.getSmallBoldFont());
         ImGui.text("Actions:");
         ImGui.popFont();
         ImGui.indent();

         renderNodeCreationClickable(relativeNode, insertionType, "Footstep Plan", FootstepPlanActionDefinition.class, null);
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
         renderNodeCreationClickable(relativeNode, insertionType, "Arm Joint Angles", ArmJointAnglesActionDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Chest Orientation", ChestOrientationActionDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Pelvis Height", PelvisHeightPitchActionDefinition.class, null);
         renderNodeCreationClickable(relativeNode, insertionType, "Wait", WaitDurationActionDefinition.class, null);
         ImGui.textDisabled("Hand Wrench: ");
         for (RobotSide side : RobotSide.values)
         {
            ImGui.sameLine();
            renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), HandWrenchActionDefinition.class, side);
         }

         ImGui.unindent();
      }
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
               // We want to to best effort initialization
               RDXActionSequence actionSequenceOrNull = null;
               if (insertionDefinition.getParent() instanceof RDXActionSequence actionSequence)
                  actionSequenceOrNull = actionSequence;
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

      insertionDefinition.getNodeToInsert().setTreeWidgetExpanded(true);
   }

   private void reindexDirectory()
   {
      indexedTreeFiles.clear();
      for (WorkspaceResourceFile queryContainedFile : treeFilesDirectory.queryContainedFiles())
      {
         indexedTreeFiles.add(new RDXAvailableBehaviorTreeFile(queryContainedFile));
      }

      // Keep them in alphabetical order
      indexedTreeFiles.sort(Comparator.comparing(RDXAvailableBehaviorTreeFile::getName));
   }
}
