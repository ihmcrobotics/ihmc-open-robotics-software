package us.ihmc.rdx.imgui;

import imgui.flag.ImGuiCond;
import imgui.internal.ImGui;

public class ImGuiTreeRenderer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public void render(long nodeID, String nodeName, Runnable renderNodeWidgets)
   {
      float indentReduction = 10.0f; // Less indent to take less space
      ImGui.unindent(indentReduction);

      boolean expanded = false;
      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
//      ImGui.setNextItemOpen(true, ImGuiCond.Once);
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
