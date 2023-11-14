package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.actions.HandPoseActionDefinition;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.Comparator;

public class RDXBehaviorTreeNodesMenu
{
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<RDXAvailableBehaviorTreeFile> indexedTreeFiles = new ArrayList<>();
   private final TypedNotification<RDXAvailableBehaviorTreeFile> loadFileRequest = new TypedNotification<>();

   public RDXBehaviorTreeNodesMenu(WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.treeFilesDirectory = treeFilesDirectory;

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

   public void renderNodeCreationWidgets(RDXBehaviorTree tree, RDXBehaviorTreeNode<?, ?> parentNode, BehaviorTreeModificationQueue modificationQueue)
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
               BehaviorTreeNodeExtension<?, ?, ?, ?> loadedNode = tree.getFileLoader().loadFromFile(indexedTreeFile, modificationQueue);

               if (tree.getRootNode() != null)
               {
                  modificationQueue.queueDestroySubtree(tree.getRootNode());
               }

               modificationQueue.queueSetRootNode(loadedNode, newRootNode -> tree.setRootNode((RDXBehaviorTreeNode<?, ?>) newRootNode));
               tree.getBehaviorTreeState().freeze();
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

      String className = ActionSequenceDefinition.class.getSimpleName();
      ImGui.text(className);
      if (ImGui.isItemHovered())
      {
         ImGuiTools.addTextUnderline(className);
      }

      ImGui.unindent();
      ImGui.separator();

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text("Actions:");
      ImGui.popFont();
      ImGui.indent();

      className = HandPoseActionDefinition.class.getSimpleName();
      ImGui.text(className);
      if (ImGui.isItemHovered())
      {
         ImGuiTools.addTextUnderline(className);
      }

      ImGui.unindent();
   }

   public TypedNotification<RDXAvailableBehaviorTreeFile> getLoadFileRequest()
   {
      return loadFileRequest;
   }
}
