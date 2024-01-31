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
      vertices.add(new Point2D32(-0.500f, 0.126f));
      vertices.add(new Point2D32(-0.268f, 0.360f));
      vertices.add(new Point2D32(0.122f, 0.326f));
      vertices.add(new Point2D32(0.276f, 0.490f));
      vertices.add(new Point2D32(0.500f, 0.270f));
      vertices.add(new Point2D32(0.351f, 0.117f));
      vertices.add(new Point2D32(0.403f, -0.215f));
      vertices.add(new Point2D32(0.133f, -0.472f));
      vertices.add(new Point2D32(-0.183f, -0.490f));
      vertices.add(new Point2D32(-0.190f, -0.324f));
      vertices.add(new Point2D32(0.067f, -0.302f));
      vertices.add(new Point2D32(0.214f, -0.147f));
      vertices.add(new Point2D32(0.174f, 0.067f));
      vertices.add(new Point2D32(0.072f, 0.159f));
      vertices.add(new Point2D32(-0.175f, 0.178f));
      vertices.add(new Point2D32(-0.326f, 0.038f));
      vertices.add(new Point2D32(-0.329f, -0.207f));
      vertices.add(new Point2D32(-0.5f, -0.22f));
      vertices.add(new Point2D32(-0.500f, 0.126f));
   }
   private final Point2D32 center = new Point2D32();
   private final ImVec2[] polygon = new ImVec2[vertices.size()];
   private int lineColor;
   private int backgroundColor;

   public ImGuiGripperWidget()
   {
      for (int i = 0; i < polygon.length; i++)
      {
         polygon[i] = new ImVec2();
      }
   }

   public void render(RobotSide side, float rowHeight)
   {
      float fontSize = ImGui.getFontSize();

      float spaceToDifferentiateLeftAndRight = 0.5f;
      if (side == RobotSide.RIGHT)
      {
         ImGui.setCursorPosX(ImGui.getCursorPosX() + fontSize * spaceToDifferentiateLeftAndRight);
      }

      float scale = 0.81f; // Make parameter if desired
      scale *= fontSize;

      center.set(0.4f * fontSize, 0.5f * fontSize);

      if (rowHeight == ImGui.getFrameHeight())
         center.addY(ImGui.getStyle().getFramePaddingY());

      float xMin = Float.MAX_VALUE;
      float xMax = Float.MIN_VALUE;
      for (int i = 0; i < vertices.size(); i++)
      {
         polygon[i].set((side.negateIfLeftSide(vertices.get(i).getX32()) * scale) + center.getX32(), (vertices.get(i).getY32() * scale) + center.getY32());

         xMin = Math.min(xMin, polygon[i].x);
         xMax = Math.max(xMax, polygon[i].x);
      }

      float itemWidth = xMax - xMin;
      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, rowHeight);

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
