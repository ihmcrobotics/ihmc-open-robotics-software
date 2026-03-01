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

   public ImGuiFallbackWidget()
   {
      for (int i = 0; i < topPolygon.length; i++)
         topPolygon[i] = new ImVec2();
      for (int i = 0; i < bottomPolygon.length; i++)
         bottomPolygon[i] = new ImVec2();
   }

   public boolean render()
   {
      return render(ImGui.getFrameHeight());
   }

   public boolean render(float lineHeight)
   {
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

      float spacing = 1.5f * ImGui.getStyle().getItemSpacingX();
      float itemWidth = xMax - xMin + spacing + scale * 0.3f;
      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + spacing);

      float cursorScreenPosX = ImGui.getCursorScreenPosX();
      float cursorScreenPosY = ImGui.getCursorScreenPosY();
      for (int i = 0; i < topPolygon.length; i++)
         topPolygon[i].set(cursorScreenPosX + topPolygon[i].x, cursorScreenPosY + topPolygon[i].y);
      for (int i = 0; i < bottomPolygon.length; i++)
         bottomPolygon[i].set(cursorScreenPosX + bottomPolygon[i].x, cursorScreenPosY + bottomPolygon[i].y);

      int hoverColor = ImGui.getColorU32(ImGuiCol.ButtonHovered);
      int lineColor = ImGui.getColorU32(ImGuiCol.Text);

      if (isHovered)
      {
         int offset = 14;
         int size = 6;
         ImVec2[] subpoly = new ImVec2[size];
         for (int i = 0; i < size; i++)
            subpoly[i] = topPolygon[(i + offset) % topPolygon.length];
         ImGui.getWindowDrawList().addConvexPolyFilled(subpoly, subpoly.length, hoverColor);

         offset = 5;
         for (int i = 0; i < size; i++)
            subpoly[i] = topPolygon[(i + offset) % topPolygon.length];
         ImGui.getWindowDrawList().addConvexPolyFilled(subpoly, subpoly.length, hoverColor);

         subpoly = new ImVec2[4];
         subpoly[0] = topPolygon[13];
         subpoly[1] = topPolygon[14];
         subpoly[2] = topPolygon[1];
         subpoly[3] = topPolygon[2];
         ImGui.getWindowDrawList().addConvexPolyFilled(subpoly, subpoly.length, hoverColor);
         subpoly[0] = topPolygon[12];
         subpoly[1] = topPolygon[13];
         subpoly[2] = topPolygon[2];
         subpoly[3] = topPolygon[3];
         ImGui.getWindowDrawList().addConvexPolyFilled(subpoly, subpoly.length, hoverColor);
         subpoly[0] = topPolygon[11];
         subpoly[1] = topPolygon[12];
         subpoly[2] = topPolygon[3];
         subpoly[3] = topPolygon[4];
         ImGui.getWindowDrawList().addConvexPolyFilled(subpoly, subpoly.length, hoverColor);
         subpoly[0] = topPolygon[10];
         subpoly[1] = topPolygon[11];
         subpoly[2] = topPolygon[4];
         subpoly[3] = topPolygon[5];
         ImGui.getWindowDrawList().addConvexPolyFilled(subpoly, subpoly.length, hoverColor);

         ImGui.getWindowDrawList().addRectFilled(bottomPolygon[0].x, bottomPolygon[0].y, bottomPolygon[2].x, bottomPolygon[2].y, hoverColor);
      }

      for (int i = 0; i < topPolygon.length - 1; i++)
         ImGui.getWindowDrawList().addLine(topPolygon[i].x, topPolygon[i].y, topPolygon[i + 1].x, topPolygon[i + 1].y, lineColor);
      for (int i = 0; i < bottomPolygon.length - 1; i++)
         ImGui.getWindowDrawList().addLine(bottomPolygon[i].x, bottomPolygon[i].y, bottomPolygon[i + 1].x, bottomPolygon[i + 1].y, lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + (itemWidth * 0.8f) + 0.1f * ImGui.getStyle().getItemSpacingX());

      ImGui.newLine();
      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }
}
