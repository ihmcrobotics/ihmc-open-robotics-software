package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.Color;
import imgui.ImFont;
import imgui.ImGui;
import imgui.ImVec2;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;

public class RDX3DPanelTooltip
{
   private final RDX3DPanel panel3D;
   private ImGui3DViewInput latestInput;
   private final Color color = new Color(0.2f, 0.2f, 0.2f, 0.7f);
   private final ImVec2 textSize = new ImVec2();

   public RDX3DPanelTooltip(RDX3DPanel panel3D)
   {
      this.panel3D = panel3D;
   }

   public void setInput(ImGui3DViewInput latestInput)
   {
      this.latestInput = latestInput;
   }

   public void render(String text)
   {
      render(text, 0);
   }

   /**
    * @param linesOfSpaceToGiveAbove For use when there is already a tooltip being displayed
    *                                above this one. This is the number of lines of text being
    *                                displayed in that one.
    */
   public void render(String text, int linesOfSpaceToGiveAbove)
   {
      if (latestInput != null)
      {
         float lineHeight = ImGui.getFrameHeight();

         // Calculate exact rendered dimensions of the text considering the font.
         ImFont font = ImGuiTools.getSmallFont();
         ImGui.pushFont(font);
         ImGui.calcTextSize(textSize, text);
         ImGui.popFont();

         float marginLeft = 10.0f;
         float marginRight = 10.0f;
         float marginTop = 10.0f;
         float marginBottom = 6.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = panel3D.getWindowDrawMinX() + mousePosX + marginLeft;
         float drawStartY = panel3D.getWindowDrawMinY() + mousePosY + marginTop + linesOfSpaceToGiveAbove * lineHeight;
         float drawEndX = drawStartX + textSize.x + marginRight;
         float drawEndY = drawStartY + textSize.y + marginBottom;

         ImGui.getWindowDrawList().addRectFilled(drawStartX, drawStartY, drawEndX, drawEndY, color.toIntBits());

         float textPositionX = drawStartX + 5.0f;
         float textPositionY = drawStartY + 2.0f;
         ImGui.getWindowDrawList().addText(font, font.getFontSize(), textPositionX, textPositionY, Color.WHITE.toIntBits(), text);
      }
   }
}
