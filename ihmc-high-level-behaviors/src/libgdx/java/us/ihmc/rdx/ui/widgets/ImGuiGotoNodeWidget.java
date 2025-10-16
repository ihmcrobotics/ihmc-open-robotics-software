package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;

/**
 * An icon for the goto node.
 */
public class ImGuiGotoNodeWidget
{
   private final ImVec2[] polygon = new ImVec2[15];

   public ImGuiGotoNodeWidget()
   {
      for (int i = 0; i < polygon.length; i++)
         polygon[i] = new ImVec2();
   }

   public void render()
   {
      float fontSize = ImGui.getFontSize();
      float scale = fontSize;

      float offsetX = ImGui.getCursorScreenPosX() + 0.0f * fontSize;
      float offsetY = ImGui.getCursorScreenPosY() + 0.0f * fontSize + ImGui.getStyle().getFramePaddingY();

      int lineColor = ImGui.getColorU32(ImGuiCol.Text);

//      polygon[0].set(0.75f, 0.18f);
//      polygon[1].set(0.95f, 0.4f);
//      polygon[2].set(0.77f, 0.4f);
//      polygon[3].set(0.77f, 0.58f);
//      polygon[4].set(0.95f, 0.78f);
//      polygon[5].set(0.67f, 0.8f);
//      polygon[6].set(0.35f, 0.76f);
//      polygon[7].set(0.21f, 0.67f);
//      polygon[8].set(0.33f, 0.59f);
//      polygon[9].set(0.33f, 0.4f);
//      polygon[10].set(0.05f, 0.4f);
//      polygon[11].set(0.25f, 0.18f);
//      polygon[12].set(0.33f, 0.59f);
//      polygon[13].set(0.33f, 0.4f);
//      polygon[14].set(0.05f, 0.4f);

      polygon[0].set(-2.687926f, 0.03162f);
      polygon[1].set(-2.529811f, -0.158113f);
      polygon[2].set(-1.6332f, -1.253726f);
      polygon[3].set(-0.327405f, -2.035031f);
      polygon[4].set(0.08561f, -1.348593f);
      polygon[5].set(-1.603495f, -0.01118f);
      polygon[6].set(0.0019f, -0.78514f);
      polygon[7].set(2.84412f, -3.863391f);
      polygon[8].set(2.838696f, 3.844866f);
      polygon[9].set(-0.01118f, 0.827613f);
      polygon[10].set(-1.650133f, -0.01853f);
      polygon[11].set(0.08752f, 1.797066f);
      polygon[12].set(1.264907f, 0.82219f);
      polygon[13].set(3.333149f, 0.03354f);
      polygon[14].set(-2.687926f, 0.03162f);
      //      polygon[1].set(-18.94445f * scale + offsetX, -1.527775f * scale + offsetY);
//      polygon[2].set(-11.81481f * scale + offsetX, -13.03704f * scale + offsetY);
//      polygon[3].set(0.2037f * scale + offsetX, -14.870369f * scale + offsetY);
//      polygon[4].set(-13.546294f * scale + offsetX, 0.203703f * scale + offsetY);
//      polygon[5].set(29.129634f * scale + offsetX, -34.527776f * scale + offsetY);
//      polygon[6].set(18.84259f * scale + offsetX, 31.370371f * scale + offsetY);
//      polygon[7].set(-17.62037f * scale + offsetX, 2.648146f * scale + offsetY);
//      polygon[8].set(0.326f * scale + offsetX, 0.500f * scale + offsetY);
//      polygon[9].set(-0.043f * scale + offsetX, 0.109f * scale + offsetY);
//      polygon[10].set(-0.256f * scale + offsetX, -0.001f * scale + offsetY);
//      polygon[11].set(-0.030f * scale + offsetX, 0.234f * scale + offsetY);
//      polygon[12].set(0.122f * scale + offsetX, 0.108f * scale + offsetY);
//      polygon[13].set(0.391f * scale + offsetX, 0.006f * scale + offsetY);
//      polygon[14].set(-0.391f * scale + offsetX, 0.005f * scale + offsetY);

      for (int i = 0; i < polygon.length - 1; i++)
         ImGui.getWindowDrawList().addLine(polygon[i].x * scale + offsetX, polygon[i].y * scale + offsetY,
                                           polygon[i + 1].x * scale + offsetX, polygon[i + 1].y * scale + offsetY, lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + scale);
      ImGui.newLine();
   }
}
