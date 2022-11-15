package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.input.ImGui3DViewInput;

public class RDX3DPanelTooltip
{
   private final RDX3DPanel panel3D;
   private ImGui3DViewInput latestInput;
   private final Color color = new Color(0.2f, 0.2f, 0.2f, 0.7f);

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
      float offsetX = 10.0f;
      float offsetY = 10.0f;
      float mousePosX = latestInput.getMousePosX();
      float mousePosY = latestInput.getMousePosY();
      float drawStartX = panel3D.getWindowDrawMinX() + mousePosX + offsetX;
      float drawStartY = panel3D.getWindowDrawMinY() + mousePosY + offsetY;

      ImGui.getWindowDrawList().addRectFilled(drawStartX, drawStartY, drawStartX + 150.0f, drawStartY + 21.0f, color.toIntBits());
      ImGui.getWindowDrawList()
           .addText(ImGuiTools.getSmallFont(),
                    ImGuiTools.getSmallFont().getFontSize(),
                    drawStartX + 5.0f,
                    drawStartY + 2.0f,
                    Color.WHITE.toIntBits(),
                    text);
   }
}
