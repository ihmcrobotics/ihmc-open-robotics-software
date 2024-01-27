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
public class ImGuiGripperWidget
{
   private final ArrayList<Point2D32> vertices = new ArrayList<>();
   {
      vertices.add(new Point2D32(2.177f, 3.823f));
      vertices.add(new Point2D32(2.442f, 4.023f));
      vertices.add(new Point2D32(2.805f, 3.941f));
      vertices.add(new Point2D32(2.956f, 4.109f));
      vertices.add(new Point2D32(3.177f, 3.909f));
      vertices.add(new Point2D32(3.005f, 3.774f));
      vertices.add(new Point2D32(3.167f, 3.471f));
      vertices.add(new Point2D32(2.934f, 3.174f));
      vertices.add(new Point2D32(2.507f, 3.195f));
      vertices.add(new Point2D32(2.545f, 3.331f));
      vertices.add(new Point2D32(2.891f, 3.390f));
      vertices.add(new Point2D32(2.859f, 3.693f));
      vertices.add(new Point2D32(2.426f, 3.806f));
      vertices.add(new Point2D32(2.388f, 3.504f));
      vertices.add(new Point2D32(2.177f, 3.823f));
   }
   private final Point2D32 center = new Point2D32();
   private final ImVec2[] polygon = new ImVec2[vertices.size()];
   private float cursorXDesktopFrame;
   private float cursorYDesktopFrame;
   private int lineColor;
   private int backgroundColor;

   public ImGuiGripperWidget()
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
         Point2D32 vertex = vertices.get(i);

         polygon[i].set((side.negateIfLeftSide(vertices.get(i).getX32()) * scale) + center.getX32(), (vertices.get(i).getY32() * scale) + center.getY32());

         xMin = Math.min(xMin, polygon[i].x);
         xMax = Math.max(xMax, polygon[i].x);
      }

      float itemWidth = xMax - xMin;

      float cursorPosX = ImGui.getCursorPosX();
      float cursorPosY = ImGui.getCursorPosY();
      cursorXDesktopFrame = ImGui.getWindowPosX() + cursorPosX - ImGui.getScrollX();
      cursorYDesktopFrame = ImGui.getWindowPosY() + cursorPosY - ImGui.getScrollY();

      for (int i = 0; i < polygon.length; i++)
      {
         polygon[i].set(cursorXDesktopFrame + polygon[i].x, cursorYDesktopFrame + polygon[i].y);
      }

      backgroundColor = side == RobotSide.LEFT ? ImGuiTools.DARK_RED : ImGuiTools.DARK_GREEN;
      lineColor = ImGui.getColorU32(ImGuiCol.Text);

//      ImGui.getWindowDrawList().addConvexPolyFilled(polygon, polygon.length, backgroundColor);

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
}
