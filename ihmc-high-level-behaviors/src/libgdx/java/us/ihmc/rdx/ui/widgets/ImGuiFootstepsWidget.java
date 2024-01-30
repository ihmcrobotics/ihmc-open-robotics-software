package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ImGuiFootstepsWidget
{
   private final Point2D32 center = new Point2D32();
   private final Point2D32 toeLeft = new Point2D32();
   private final Point2D32 toeRight = new Point2D32();
   private final Point2D32 topLeft = new Point2D32();
   private final Point2D32 topRight = new Point2D32();
   private final Point2D32 bottomLeft = new Point2D32();
   private final Point2D32 bottomRight = new Point2D32();
   private final Point2D32 heelLeft = new Point2D32();
   private final Point2D32 heelRight = new Point2D32();
   private final ImVec2[] polygon = new ImVec2[9];
   private int lineColor;
   private int backgroundColor;
   private final SideDependentList<Boolean> isHovered = new SideDependentList<>(false, false);

   public ImGuiFootstepsWidget()
   {
      for (int i = 0; i < polygon.length; i++)
      {
         polygon[i] = new ImVec2();
      }
   }

   public void render(float rowHeight)
   {
      render(RobotSide.LEFT, rowHeight);
      float fontSize = ImGui.getFontSize();
      ImGui.sameLine();
      ImGui.setCursorPosX(ImGui.getCursorPosX() - fontSize * 0.2f);
      render(RobotSide.RIGHT, rowHeight);
   }

   public void render(RobotSide side, float rowHeight)
   {
      float fontSize = ImGui.getFontSize();

      float scale = 0.6f; // Make parameter if desired
      scale *= fontSize;

      center.set(0.3f * fontSize, 0.5f * fontSize);

      if (rowHeight == ImGui.getFrameHeight())
         center.addY(ImGui.getStyle().getFramePaddingY());

      float halfWidth = 0.34f;
      float halfHeight = 0.7f;
      float toeHalfWidth = 0.17f;
      float toeHeight = 0.26f;
      float heelHeight = 0.1f;
      float heelHalfWidth = 0.26f;

      toeLeft.set(-toeHalfWidth, -halfHeight);
      toeRight.set(toeHalfWidth, -halfHeight);
      topLeft.set(-halfWidth, -halfHeight + toeHeight);
      topRight.set(halfWidth, -halfHeight + toeHeight);
      bottomLeft.set(-halfWidth, halfHeight - heelHeight);
      bottomRight.set(halfWidth, halfHeight - heelHeight);
      heelLeft.set(-heelHalfWidth, halfHeight);
      heelRight.set(heelHalfWidth, halfHeight);

      toeLeft.scaleAdd(scale, center);
      toeRight.scaleAdd(scale, center);
      topLeft.scaleAdd(scale, center);
      topRight.scaleAdd(scale, center);
      bottomLeft.scaleAdd(scale, center);
      bottomRight.scaleAdd(scale, center);
      heelLeft.scaleAdd(scale, center);
      heelRight.scaleAdd(scale, center);

      float itemWidth = bottomRight.getX32() - bottomLeft.getX32();
      isHovered.set(side, ImGuiTools.isItemHovered(itemWidth));

      float cursorScreenPosX = ImGui.getCursorScreenPosX();
      float cursorScreenPosY = ImGui.getCursorScreenPosY();
      polygon[0].set(cursorScreenPosX + bottomRight.getX32(), cursorScreenPosY + bottomRight.getY32());
      polygon[1].set(cursorScreenPosX + heelRight.getX32(), cursorScreenPosY + heelRight.getY32());
      polygon[2].set(cursorScreenPosX + heelLeft.getX32(), cursorScreenPosY + heelLeft.getY32());
      polygon[3].set(cursorScreenPosX + bottomLeft.getX32(), cursorScreenPosY + bottomLeft.getY32());
      polygon[4].set(cursorScreenPosX + topLeft.getX32(), cursorScreenPosY + topLeft.getY32());
      polygon[5].set(cursorScreenPosX + toeLeft.getX32(), cursorScreenPosY + toeLeft.getY32());
      polygon[6].set(cursorScreenPosX + toeRight.getX32(), cursorScreenPosY + toeRight.getY32());
      polygon[7].set(cursorScreenPosX + topRight.getX32(), cursorScreenPosY + topRight.getY32());
      polygon[8].set(cursorScreenPosX + bottomRight.getX32(), cursorScreenPosY + bottomRight.getY32());

      backgroundColor = side == RobotSide.LEFT ? ImGuiTools.DARK_RED : ImGuiTools.DARK_GREEN;
      lineColor = ImGui.getColorU32(ImGuiCol.Text);

      if (isHovered.get(side))
         ImGui.getWindowDrawList().addConvexPolyFilled(polygon, polygon.length, backgroundColor);

      for (int i = 0; i < polygon.length - 1; i++)
      {
         drawLine(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
      }

      ImGui.setCursorPosX(ImGui.getCursorPosX() + (itemWidth * 1.1f));

      ImGui.newLine();
   }

   private void drawLine(float x0, float y0, float x1, float y1)
   {
      ImGui.getWindowDrawList().addLine(x0, y0, x1, y1, lineColor);
   }

   public SideDependentList<Boolean> getIsHovered()
   {
      return isHovered;
   }
}
