package us.ihmc.rdx.behaviorTree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.ImGuiTools;

public class RDXBehaviorTreeTimeline
{

   public static void render(RDXBehaviorTreeRootNode rootNode)
   {

   }

   public static float heightNeeded()
   {
      return 100.0f;
   }

   public static boolean renderIcon(boolean selected)
   {
      float height = ImGui.getFrameHeight();
      float scale = ImGui.getFontSize();
      float itemWidth = 2.0f * scale;
      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, height);


      int lineColor = ImGui.getColorU32(ImGuiCol.Text);


      float offsetX = ImGui.getCursorScreenPosX();
      float offsetY = ImGui.getCursorScreenPosY();

      if (isHovered || selected)
      {
         ImGui.getWindowDrawList().addRectFilled(offsetX, offsetY, offsetX + 0.9f * scale, offsetY + 0.3f * scale, ImGuiTools.LIGHT_BLUE);
         ImGui.getWindowDrawList().addRectFilled(offsetX + 0.35f * scale, offsetY + 0.3f * scale,
                                           offsetX + (0.9f + 0.35f) * scale, offsetY + 2f * 0.3f * scale, ImGuiTools.DARK_GREEN);
         ImGui.getWindowDrawList().addRectFilled(offsetX + (2f * 0.35f) * scale, offsetY + (2f * 0.3f) * scale,
                                           offsetX + 1.5f * scale, offsetY + 3f * 0.3f * scale, ImGuiTools.YELLOW);
      }

      ImGui.getWindowDrawList().addRect(offsetX, offsetY, offsetX + 0.9f * scale, offsetY + 0.3f * scale, lineColor);
      ImGui.getWindowDrawList().addRect(offsetX + 0.35f * scale, offsetY + 0.3f * scale,
                                        offsetX + (0.9f + 0.35f) * scale, offsetY + 2f * 0.3f * scale, lineColor);
      ImGui.getWindowDrawList().addRect(offsetX + (2f * 0.35f) * scale, offsetY + (2f * 0.3f) * scale,
                                        offsetX + 1.5f * scale, offsetY + 3f * 0.3f * scale, lineColor);


      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);
      ImGui.newLine();

      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }
}
