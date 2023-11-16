package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionDefinition;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.behaviors.sequence.actions.WalkActionDefinition;
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

public class RDXBehaviorTreeNodesMenu
{
   private final RDXBehaviorTree tree;
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final BehaviorTreeTopologyOperationQueue topologyOperationQueue;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<RDXAvailableBehaviorTreeFile> indexedTreeFiles = new ArrayList<>();

   public RDXBehaviorTreeNodesMenu(RDXBehaviorTree tree, WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.tree = tree;
      this.treeFilesDirectory = treeFilesDirectory;

      topologyOperationQueue = tree.getBehaviorTreeState().getTopologyChangeQueue();

      reindexDirectory();
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

   /**
    * This method assumes that the insertion is valid for the relative node.
    * For example, if the insertion requires modifying a parent, we assume it is not null.
    */
   public void renderNodeCreationWidgets(RDXBehaviorTreeNode<?, ?> relativeNode, BehaviorTreeNodeInsertionType insertionType)
   {
      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("From file:");
      ImGui.popFont();

      ImGui.indent();
      for (RDXAvailableBehaviorTreeFile indexedTreeFile : indexedTreeFiles)
      {
         String fileName = indexedTreeFile.getTreeFile().getFileName();
         ImGui.text(fileName);

         if (ImGui.isItemHovered())
         {
            ImGuiTools.addTextUnderline(fileName);

            if (ImGui.isMouseDoubleClicked(ImGuiMouseButton.Left))
            {
               RDXBehaviorTreeNode<?, ?> loadedNode = tree.getFileLoader().loadFromFile(indexedTreeFile, topologyOperationQueue);

               BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                   = BehaviorTreeNodeInsertionDefinition.build(loadedNode, tree.getBehaviorTreeState(), tree::setRootNode, relativeNode, insertionType);

               topologyOperationQueue.queueInsertNode(insertionDefinition);
               ImGui.closeCurrentPopup();
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

      renderNodeCreationClickable(relativeNode, insertionType, "Action Sequence", ActionSequenceDefinition.class, null);

      ImGui.unindent();
      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("Actions:");
      ImGui.popFont();
      ImGui.indent();

      renderNodeCreationClickable(relativeNode, insertionType, "Walk Action", WalkActionDefinition.class, null);
      ImGui.text("Hand Pose: ");
      for (RobotSide side : RobotSide.values)
      {
         renderNodeCreationClickable(relativeNode, insertionType, side.getPascalCaseName(), HandPoseActionDefinition.class, side);
      }

      ImGui.unindent();
   }

   private void renderNodeCreationClickable(RDXBehaviorTreeNode<?, ?> relativeNode,
                                            BehaviorTreeNodeInsertionType insertionType,
                                            String nodeTypeName,
                                            Class<?> nodeType,
                                            @Nullable RobotSide side)
   {
      ImGui.text(nodeTypeName);
      if (ImGui.isItemHovered())
      {
         ImGuiTools.addTextUnderline(nodeTypeName);

         if (ImGui.isMouseDoubleClicked(ImGuiMouseButton.Left))
         {
            RDXBehaviorTreeNode<?, ?> newNode = tree.getNodeBuilder()
                                                    .createNode(nodeType,
                                                                tree.getBehaviorTreeState().getAndIncrementNextID(),
                                                                tree.getBehaviorTreeState().getCRDTInfo());


            BehaviorTreeNodeInsertionDefinition<RDXBehaviorTreeNode<?, ?>> insertionDefinition
                  = BehaviorTreeNodeInsertionDefinition.build(newNode, tree.getBehaviorTreeState(), tree::setRootNode, relativeNode, insertionType);

            if (insertionDefinition.getNodeToInsert() instanceof RDXActionNode<?, ?> newAction
             && insertionDefinition.getParent() instanceof RDXActionSequence actionSequence)
            {
               tree.getNodeBuilder().initializeActionNode(actionSequence, newAction, insertionDefinition.getInsertionIndex(), side);
            }

            topologyOperationQueue.queueInsertNode(insertionDefinition);
            ImGui.closeCurrentPopup();
         }
      }
   }
}
