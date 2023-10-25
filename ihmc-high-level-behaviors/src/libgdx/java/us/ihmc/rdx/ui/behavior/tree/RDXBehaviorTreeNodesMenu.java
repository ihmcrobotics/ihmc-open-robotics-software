package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXBehaviorTreeNodesMenu
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final Notification menuShouldClose = new Notification();

   public void renderNodesMenu()
   {
      if (ImGui.beginMenu(labels.get("Nodes"), !menuShouldClose.poll()))
      {
         // TODO: Sections for "From File", "Control Nodes", "Actions"

         ImGui.endMenu();
      }
   }
}
