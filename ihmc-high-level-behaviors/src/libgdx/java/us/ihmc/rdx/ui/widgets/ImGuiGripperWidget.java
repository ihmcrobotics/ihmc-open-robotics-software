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
      vertices.add(new Point2D32(-0.500f, 0.181f));
      vertices.add(new Point2D32(-0.235f, 0.381f));
      vertices.add(new Point2D32(0.127f, 0.300f));
      vertices.add(new Point2D32(0.278f, 0.468f));
      vertices.add(new Point2D32(0.500f, 0.268f));
      vertices.add(new Point2D32(0.327f, 0.132f));
      vertices.add(new Point2D32(0.489f, -0.170f));
      vertices.add(new Point2D32(0.257f, -0.468f));
      vertices.add(new Point2D32(-0.170f, -0.446f));
      vertices.add(new Point2D32(-0.132f, -0.311f));
      vertices.add(new Point2D32(0.214f, -0.251f));
      vertices.add(new Point2D32(0.181f, 0.051f));
      vertices.add(new Point2D32(-0.251f, 0.165f));
      vertices.add(new Point2D32(-0.289f, -0.138f));
      vertices.add(new Point2D32(-0.500f, 0.181f));
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

      ImGui.setCursorPosX(ImGui.getCursorPosX() + (itemWidth * 0.8f));

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
