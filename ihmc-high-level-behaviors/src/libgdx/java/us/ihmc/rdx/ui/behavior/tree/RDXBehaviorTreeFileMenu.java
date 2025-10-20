package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

import javax.annotation.Nullable;

public class RDXBehaviorTreeFileMenu
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public void renderFileMenu(@Nullable RDXBehaviorTreeNode<?, ?> rootNode, RDXBehaviorTreeNodeCreationMenu nodeCreationMenu)
   {
      if (ImGui.beginMenu(labels.get("File")))
      {
         if (rootNode != null)
         {
            if (ImGui.menuItem(labels.get("Save"), "Ctrl + S"))
            {
               RDXBaseUI.pushNotification("Saving %s".formatted(rootNode.getDefinition().getName()));
               rootNode.getDefinition().saveToFile();
            }
            if (ImGui.menuItem(labels.get("Undo All Non-topological Changes")))
            {
               RDXBaseUI.pushNotification("Undoing all non-topological behavior tree changes");
               rootNode.getDefinition().undoAllNontopologicalChanges();
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
