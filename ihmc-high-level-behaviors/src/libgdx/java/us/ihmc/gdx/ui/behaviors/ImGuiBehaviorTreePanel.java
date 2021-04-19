package us.ihmc.gdx.ui.behaviors;

import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.ImGuiTools;

public class ImGuiBehaviorTreePanel
{
   private final String windowName;

   public ImGuiBehaviorTreePanel(String name)
   {
      windowName = ImGuiTools.uniqueLabel(getClass(), name + " tree");
   }

   public void renderAsWindow()
   {
      ImGui.begin(windowName);
      renderWidgetsOnly();
      ImGui.end();
   }

   public void renderWidgetsOnly()
   {
//      ImGui.text("Behavior tree render WIP");

      float x = ImGui.getItemRectMinX();
      float y = ImGui.getItemRectMaxY();
      int color = ImGui.colorConvertFloat4ToU32(0.5f, 0.5f, 0.5f, 1.0f);
      ImGui.getWindowDrawList().addCircle(x + 30.0f, y + 30.0f, 15.0f, color);

      ImGui.getWindowDrawList().addRect(x + 50.0f, y, x + 100.0f, y + 50.0f, ImGui.getColorU32(0.5f, 0.5f, 0.5f, 1.0f));
   }
}
