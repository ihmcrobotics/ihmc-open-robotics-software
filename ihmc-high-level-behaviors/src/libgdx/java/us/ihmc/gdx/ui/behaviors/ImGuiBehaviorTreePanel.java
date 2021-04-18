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

   public void render()
   {
      ImGui.begin(windowName);

      ImGui.text("Hello");

      float x = ImGui.getWindowPosX();
      float y = ImGui.getWindowPosY();
      int color = ImGui.colorConvertFloat4ToU32(0.5f, 0.5f, 0.5f, 1.0f);
      ImGui.getWindowDrawList().addCircle(x + 50.0f, y + 50.0f, 3.0f, color);

      ImGui.getWindowDrawList().addRect(x, y, x + 50.0f, y + 50.0f, ImGui.getColorU32(0.5f, 0.5f, 0.5f, 1.0f));

      ImGui.end();
   }
}
