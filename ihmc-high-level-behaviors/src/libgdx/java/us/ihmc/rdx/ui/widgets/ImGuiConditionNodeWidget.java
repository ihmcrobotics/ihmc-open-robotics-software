package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.rdx.imgui.ImGuiTools;

import java.util.ArrayList;

/**
 * A clickable behavior root node icon with hover state.
 */
public class ImGuiConditionNodeWidget
{
   private final ArrayList<Point2D32> checkmark = new ArrayList<>();
   {
      checkmark.add(new Point2D32(-0.143f, 0.405f));
      checkmark.add(new Point2D32(0.500f, -0.285f));
      checkmark.add(new Point2D32(0.375f, -0.405f));
      checkmark.add(new Point2D32(-0.157f, 0.146f));
      checkmark.add(new Point2D32(-0.368f, -0.056f));
      checkmark.add(new Point2D32(-0.500f, 0.103f));
      checkmark.add(new Point2D32(-0.143f, 0.405f));
   }
   private final ImVec2[] checkPolygon = new ImVec2[checkmark.size()];
   private final ImVec2[] xPolygon = new ImVec2[13];

   public ImGuiConditionNodeWidget()
   {
      for (int i = 0; i < checkPolygon.length; i++)
         checkPolygon[i] = new ImVec2();
      for (int i = 0; i < xPolygon.length; i++)
         xPolygon[i] = new ImVec2();
   }

   public boolean render()
   {
      return render(ImGui.getFrameHeight());
   }

   public boolean render(float lineHeight)
   {
      float fontSize = ImGui.getFontSize();
      float scale = 0.5f * fontSize;

      float offsetX = ImGui.getCursorScreenPosX() + 0.3f * fontSize;
      float offsetY = ImGui.getCursorScreenPosY() + 0.41f * fontSize;
      if (lineHeight == ImGui.getFrameHeight())
         offsetY += ImGui.getStyle().getFramePaddingY();

      int lineColor = ImGui.getColorU32(ImGuiCol.Text);

      float xMin = Float.MAX_VALUE;
      float xMax = Float.MIN_VALUE;
      for (int i = 0; i < checkmark.size(); i++)
      {
         checkPolygon[i].set((checkmark.get(i).getX32() * scale) + offsetX, (checkmark.get(i).getY32() * scale) + offsetY);

         xMin = Math.min(xMin, checkPolygon[i].x);
         xMax = Math.max(xMax, checkPolygon[i].x);
      }

      xPolygon[0].set(-0.185f, -0.003f);
      xPolygon[1].set(-0.495f, 0.313f);
      xPolygon[2].set(-0.325f, 0.500f);
      xPolygon[3].set(-0.008f, 0.165f);
      xPolygon[4].set(0.314f, 0.479f);
      xPolygon[5].set(0.495f, 0.271f);
      xPolygon[6].set(0.187f, -0.018f);
      xPolygon[7].set(0.483f, -0.321f);
      xPolygon[8].set(0.317f, -0.500f);
      xPolygon[9].set(-0.001f, -0.181f);
      xPolygon[10].set(-0.316f, -0.495f);
      xPolygon[11].set(-0.495f, -0.271f);
      xPolygon[12].set(-0.185f, -0.003f);

      offsetX += 0.5f * fontSize;
      offsetY += 0.3f * fontSize;
      scale = 0.4f * fontSize;
      for (int i = 0; i < xPolygon.length; i++)
      {
         xPolygon[i].set((xPolygon[i].x * scale) + offsetX, (xPolygon[i].y * scale) + offsetY);

         xMin = Math.min(xMin, xPolygon[i].x);
         xMax = Math.max(xMax, xPolygon[i].x);
      }

      float itemWidth = xMax - xMin;
      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);

      for (int i = 0; i < checkPolygon.length - 1; i++)
         ImGui.getWindowDrawList().addLine(checkPolygon[i].x, checkPolygon[i].y, checkPolygon[i + 1].x, checkPolygon[i + 1].y, lineColor);
      for (int i = 0; i < xPolygon.length - 1; i++)
         ImGui.getWindowDrawList().addLine(xPolygon[i].x, xPolygon[i].y, xPolygon[i + 1].x, xPolygon[i + 1].y, lineColor);
      ImGui.getWindowDrawList().addLine(-1.6f * scale + offsetX, 0.5f * scale + offsetY, 0.1f * scale + offsetX, -1.3f * scale + offsetY, lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);
      ImGui.newLine();
      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }
}
