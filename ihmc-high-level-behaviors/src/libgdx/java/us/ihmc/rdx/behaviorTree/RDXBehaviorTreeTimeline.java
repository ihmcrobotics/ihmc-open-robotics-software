package us.ihmc.rdx.behaviorTree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;

public class RDXBehaviorTreeTimeline extends RDXPanel
{
   private RDXBehaviorTreeRootNode rootNode;

   public RDXBehaviorTreeTimeline()
   {
      super("Behavior Timeline");

      setRenderMethod(this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {
      if (rootNode == null)
      {
         ImGui.text("Root node is null. No scene.");
         return;
      }

      // TODO render timeline


   }

   public static boolean renderIcon()
   {
      float height = ImGui.getFrameHeight();
      float scale = ImGui.getFontSize();
      float itemWidth = 2.0f * scale;
      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, height);


      int lineColor = ImGui.getColorU32(ImGuiCol.Text);


      float offsetX = ImGui.getCursorScreenPosX();
      float offsetY = ImGui.getCursorScreenPosY();

      if (isHovered)
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

   public void setRootNode(RDXBehaviorTreeRootNode rootNode)
   {
      this.rootNode = rootNode;
   }
}
