package us.ihmc.rdx.imgui;

import imgui.ImGui;
import us.ihmc.commons.thread.TypedNotification;

public class ImGuiTreeRenderer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public void render(long nodeID, String nodeName, Runnable renderNodeWidgets)
   {
      render(nodeID, nodeName, renderNodeWidgets, null);
   }

   /**
    * @param expandCollapseRequest Allows a node to pass in a request to expand/collapse it's tree node
    */
   public void render(long nodeID, String nodeName, Runnable renderNodeWidgets, TypedNotification<Boolean> expandCollapseRequest)
   {
      float indentReduction = 10.0f; // Less indent to take less space
      ImGui.unindent(indentReduction);

      boolean expanded = false;
      ImGui.pushFont(ImGuiTools.getSmallBoldFont());

      if (expandCollapseRequest != null && expandCollapseRequest.poll())
         ImGui.setNextItemOpen(expandCollapseRequest.read());

      if (ImGui.treeNode(labels.getHidden(Long.toString(nodeID)), nodeName))
      {
         expanded = true;
         ImGui.popFont();

         renderNodeWidgets.run();

         ImGui.treePop();
      }

      if (!expanded)
         ImGui.popFont();

      ImGui.indent(indentReduction);
   }
}
