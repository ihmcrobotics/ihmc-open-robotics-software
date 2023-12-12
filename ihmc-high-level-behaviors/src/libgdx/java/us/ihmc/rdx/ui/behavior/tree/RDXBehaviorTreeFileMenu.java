package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXBehaviorTreeFileMenu
{
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final Notification menuShouldClose = new Notification();

   public RDXBehaviorTreeFileMenu(WorkspaceResourceDirectory treeFilesDirectory)
   {
      this.treeFilesDirectory = treeFilesDirectory;
   }

   public void renderFileMenu()
   {
      if (ImGui.beginMenu(labels.get("File"), !menuShouldClose.poll()))
      {

         // TODO: Iterate through tree finding nodes that coorespond to JSON files.

         // TODO: Probably some widgets in here that manage whether the selected node
         //   currently cooresponds to a JSON file and the path to the file

         ImGui.endMenu();
      }
   }
}
