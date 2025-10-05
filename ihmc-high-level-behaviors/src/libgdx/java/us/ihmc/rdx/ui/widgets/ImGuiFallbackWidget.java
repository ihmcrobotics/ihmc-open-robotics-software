package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.rdx.imgui.ImGuiTools;

import java.util.ArrayList;

/**
 * Use ImGuiSVGWidgetNormalizer to edit.
 */
public class ImGuiFallbackWidget
{
   private final ArrayList<Point2D32> topPart = new ArrayList<>();
   {
      topPart.add(new Point2D32(0.106f, 0.497f));
      topPart.add(new Point2D32(0.176f, 0.157f));
      topPart.add(new Point2D32(0.332f, 0.070f));
      topPart.add(new Point2D32(0.419f, -0.104f));
      topPart.add(new Point2D32(0.388f, -0.318f));
      topPart.add(new Point2D32(0.110f, -0.500f));
      topPart.add(new Point2D32(-0.180f, -0.497f));
      topPart.add(new Point2D32(-0.347f, -0.355f));
      topPart.add(new Point2D32(-0.419f, -0.175f));
      topPart.add(new Point2D32(-0.249f, -0.094f));
      topPart.add(new Point2D32(-0.127f, -0.294f));
      topPart.add(new Point2D32(0.100f, -0.297f));
      topPart.add(new Point2D32(0.206f, -0.186f));
      topPart.add(new Point2D32(0.169f, -0.056f));
      topPart.add(new Point2D32(0.011f, 0.022f));
      topPart.add(new Point2D32(-0.134f, 0.172f));
      topPart.add(new Point2D32(-0.134f, 0.500f));
      topPart.add(new Point2D32(0.106f, 0.497f));
   }
   private final ArrayList<Point2D32> bottomPart = new ArrayList<>();
   {
      float size = 0.16f;
      float offset = 0.8f;
      bottomPart.add(new Point2D32(-size, -size + offset));
      bottomPart.add(new Point2D32(size, -size + offset));
      bottomPart.add(new Point2D32(size, size + offset));
      bottomPart.add(new Point2D32(-size, size + offset));
      bottomPart.add(new Point2D32(-size, -size + offset));
   }
   private final Point2D32 center = new Point2D32();
   private final ImVec2[] topPolygon = new ImVec2[topPart.size()];
   private final ImVec2[] bottomPolygon = new ImVec2[bottomPart.size()];
   private int lineColor;

   public ImGuiFallbackWidget()
   {
      for (int i = 0; i < topPolygon.length; i++)
         topPolygon[i] = new ImVec2();
      for (int i = 0; i < bottomPolygon.length; i++)
         bottomPolygon[i] = new ImVec2();
   }

   public boolean render()
   {
      float lineHeight = ImGui.getFrameHeight();
      float fontSize = ImGui.getFontSize();

      float scale = 0.7f; // Make parameter if desired
      scale *= fontSize;

      center.set(0.3f * fontSize, 0.35f * fontSize);

      if (lineHeight == ImGui.getFrameHeight())
         center.addY(ImGui.getStyle().getFramePaddingY());

      float xMin = Float.MAX_VALUE;
      float xMax = Float.MIN_VALUE;
      for (int i = 0; i < topPart.size(); i++)
      {
         topPolygon[i].set((topPart.get(i).getX32() * scale) + center.getX32(), (topPart.get(i).getY32() * scale) + center.getY32());

         xMin = Math.min(xMin, topPolygon[i].x);
         xMax = Math.max(xMax, topPolygon[i].x);
      }
      for (int i = 0; i < bottomPart.size(); i++)
      {
         bottomPolygon[i].set((bottomPart.get(i).getX32() * scale) + center.getX32(), (bottomPart.get(i).getY32() * scale) + center.getY32());

         xMin = Math.min(xMin, bottomPolygon[i].x);
         xMax = Math.max(xMax, bottomPolygon[i].x);
      }

      float itemWidth = xMax - xMin;
      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);

      float cursorScreenPosX = ImGui.getCursorScreenPosX();
      float cursorScreenPosY = ImGui.getCursorScreenPosY();
      for (int i = 0; i < topPolygon.length; i++)
         topPolygon[i].set(cursorScreenPosX + topPolygon[i].x, cursorScreenPosY + topPolygon[i].y);
      for (int i = 0; i < bottomPolygon.length; i++)
         bottomPolygon[i].set(cursorScreenPosX + bottomPolygon[i].x, cursorScreenPosY + bottomPolygon[i].y);

      lineColor = isHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Text); // TODO: Need to fill instead

      for (int i = 0; i < topPolygon.length - 1; i++)
         drawLine(topPolygon[i].x, topPolygon[i].y, topPolygon[i + 1].x, topPolygon[i + 1].y);
      for (int i = 0; i < bottomPolygon.length - 1; i++)
         drawLine(bottomPolygon[i].x, bottomPolygon[i].y, bottomPolygon[i + 1].x, bottomPolygon[i + 1].y);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + (itemWidth * 0.8f));

      ImGui.newLine();
      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }

   private void drawLine(float x0, float y0, float x1, float y1)
   {
      ImGui.getWindowDrawList().addLine(x0, y0, x1, y1, lineColor);
   }
}
