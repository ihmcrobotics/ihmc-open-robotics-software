package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class RDXBehaviorTreeFileMenu
{
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final Notification menuShouldClose = new Notification();

   public RDXBehaviorTreeFileMenu(WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.treeFilesDirectory = treeFilesDirectory;
   }

   public void renderFileMenu(@Nullable RDXBehaviorTreeNode<?, ?> rootNode, RDXBehaviorTreeNodeCreationMenu nodeCreationMenu)
   {
      if (ImGui.beginMenu(labels.get("File"), !menuShouldClose.poll()))
      {
         if (rootNode != null)
         {
            if (ImGui.menuItem(labels.get("Save"), "Ctrl + S"))
            {
               RDXBaseUI.pushNotification("Saving %s".formatted(rootNode.getDefinition().getName()));
               rootNode.getDefinition().saveToFile();
            }
         }
         else
         {
            if (ImGui.menuItem(labels.get("Refresh File List")))
            {
               nodeCreationMenu.reindexDirectory();
            }
         }

         ImGui.endMenu();
      }
   }
}
