package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

/**
 * Use ImGuiSVGWidgetNormalizer to edit.
 */
public class ImGuiHandWidget
{
   private final ArrayList<Point2D32> vertices = new ArrayList<>();
   private boolean isHovered;

   {
      vertices.add(new Point2D32(-0.32f, 0.4f));
      vertices.add(new Point2D32(-0.160f, 0.495f));
      vertices.add(new Point2D32(-0.007f, 0.500f));
      vertices.add(new Point2D32(0.133f, 0.467f));
      vertices.add(new Point2D32(0.199f, 0.385f));
      vertices.add(new Point2D32(0.264f, 0.237f));
      vertices.add(new Point2D32(0.327f, 0.045f));
      vertices.add(new Point2D32(0.341f, -0.053f));
      vertices.add(new Point2D32(0.234f, -0.092f));
      vertices.add(new Point2D32(0.188f, -0.048f));
      vertices.add(new Point2D32(0.179f, 0.062f));
      vertices.add(new Point2D32(0.174f, 0.130f));
      vertices.add(new Point2D32(0.149f, -0.426f));
      vertices.add(new Point2D32(0.051f, -0.396f));
      vertices.add(new Point2D32(0.053f, -0.336f));
      vertices.add(new Point2D32(0.048f, -0.001f));
      vertices.add(new Point2D32(0.042f, -0.396f));
      vertices.add(new Point2D32(0.015f, -0.486f));
      vertices.add(new Point2D32(-0.045f, -0.470f));
      vertices.add(new Point2D32(-0.078f, -0.396f));
      vertices.add(new Point2D32(-0.070f, -0.015f));
      vertices.add(new Point2D32(-0.122f, -0.500f));
      vertices.add(new Point2D32(-0.201f, -0.484f));
      vertices.add(new Point2D32(-0.207f, -0.426f));
      vertices.add(new Point2D32(-0.201f, -0.034f));
      vertices.add(new Point2D32(-0.212f, -0.388f));
      vertices.add(new Point2D32(-0.237f, -0.445f));
      vertices.add(new Point2D32(-0.303f, -0.440f));
      vertices.add(new Point2D32(-0.341f, -0.352f));
      vertices.add(new Point2D32(-0.32f, 0.4f));
   }
   private final Point2D32 center = new Point2D32();
   private final ImVec2[] polygon = new ImVec2[vertices.size()];
   private int lineColor;
   private int backgroundColor;

   public ImGuiHandWidget()
   {
      for (int i = 0; i < polygon.length; i++)
      {
         polygon[i] = new ImVec2();
      }
   }

   public void render(RobotSide side)
   {
      float fontSize = ImGui.getFontSize();

      float spaceToDifferentiateLeftAndRight = 0.5f;
      if (side == RobotSide.RIGHT)
      {
         ImGui.setCursorPosX(ImGui.getCursorPosX() + fontSize * spaceToDifferentiateLeftAndRight);
      }

      float scale = 0.85f; // Make parameter if desired
      scale *= fontSize;

      center.set(0.3f * fontSize, 0.5f * fontSize);

      float xMin = Float.MAX_VALUE;
      float xMax = Float.MIN_VALUE;
      for (int i = 0; i < vertices.size(); i++)
      {
         polygon[i].set((side.negateIfRightSide(vertices.get(i).getX32()) * scale) + center.getX32(), (vertices.get(i).getY32() * scale) + center.getY32());

         xMin = Math.min(xMin, polygon[i].x);
         xMax = Math.max(xMax, polygon[i].x);
      }

      float itemWidth = xMax - xMin;
      isHovered = ImGuiTools.isItemHovered(itemWidth);

      float cursorScreenPosX = ImGui.getCursorScreenPosX();
      float cursorScreenPosY = ImGui.getCursorScreenPosY();
      for (int i = 0; i < polygon.length; i++)
      {
         polygon[i].set(cursorScreenPosX + polygon[i].x, cursorScreenPosY + polygon[i].y);
      }

      backgroundColor = side == RobotSide.LEFT ? ImGuiTools.DARK_RED : ImGuiTools.DARK_GREEN;
      lineColor = ImGui.getColorU32(ImGuiCol.Text);

      if (isHovered)
         ImGui.getWindowDrawList().addConvexPolyFilled(polygon, polygon.length, backgroundColor);

      for (int i = 0; i < polygon.length - 1; i++)
      {
         drawLine(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
      }

      ImGui.setCursorPosX(ImGui.getCursorPosX() + (itemWidth * 1.2f));

      if (side == RobotSide.LEFT)
      {
         ImGui.setCursorPosX(ImGui.getCursorPosX() + fontSize * spaceToDifferentiateLeftAndRight);
      }

      ImGui.newLine();
   }

   private void drawLine(float x0, float y0, float x1, float y1)
   {
      ImGui.getWindowDrawList().addLine(x0, y0, x1, y1, lineColor);
   }

   public boolean getIsHovered()
   {
      return isHovered;
   }
}
