package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class ImGuiFootstepsWidget
{
   private final Point2D32 center = new Point2D32();
   private final Point2D32 toeLeft = new Point2D32();
   private final Point2D32 toeRight = new Point2D32();
   private final Point2D32 topLeft = new Point2D32();
   private final Point2D32 topRight = new Point2D32();
   private final Point2D32 bottomLeft = new Point2D32();
   private final Point2D32 bottomRight = new Point2D32();
   private final ImVec2[] polygon = new ImVec2[6];
   private float cursorXDesktopFrame;
   private float cursorYDesktopFrame;
   private int lineColor;
   private int backgroundColor;
   private boolean isHovered = false;

   public ImGuiFootstepsWidget()
   {
      for (int i = 0; i < polygon.length; i++)
      {
         polygon[i] = new ImVec2();
      }
   }

   public void render()
   {
      render(RobotSide.LEFT);
      float fontSize = ImGui.getFontSize();
      ImGui.sameLine();
      ImGui.setCursorPosX(ImGui.getCursorPosX() - fontSize * 0.2f);
      render(RobotSide.RIGHT);
   }

   public void render(RobotSide side)
   {

      float fontSize = ImGui.getFontSize();

      float scale = 0.7f; // Make parameter if desired
      scale *= fontSize;

      center.set(0.3f * fontSize, 0.5f * fontSize);

      float halfWidth = 0.34f;
      float halfHeight = 0.7f;
      float toeHalfWidth = 0.17f;
      float toeHeight = 0.22f;

      toeLeft.set(-toeHalfWidth, -halfHeight);
      toeRight.set(toeHalfWidth, -halfHeight);
      topLeft.set(-halfWidth, -halfHeight + toeHeight);
      topRight.set(halfWidth, -halfHeight + toeHeight);
      bottomLeft.set(-halfWidth, halfHeight);
      bottomRight.set(halfWidth, halfHeight);

      toeLeft.scaleAdd(scale, center);
      toeRight.scaleAdd(scale, center);
      topLeft.scaleAdd(scale, center);
      topRight.scaleAdd(scale, center);
      bottomLeft.scaleAdd(scale, center);
      bottomRight.scaleAdd(scale, center);

      float itemWidth = bottomRight.getX32() - bottomLeft.getX32();
      isHovered = ImGuiTools.isItemHovered(itemWidth);

      float cursorPosX = ImGui.getCursorPosX();
      float cursorPosY = ImGui.getCursorPosY();
      cursorXDesktopFrame = ImGui.getWindowPosX() + cursorPosX - ImGui.getScrollX();
      cursorYDesktopFrame = ImGui.getWindowPosY() + cursorPosY - ImGui.getScrollY();

      backgroundColor = side == RobotSide.LEFT ? ImGuiTools.DARK_RED : ImGuiTools.DARK_GREEN;

      polygon[0].set(cursorXDesktopFrame + bottomRight.getX32(), cursorYDesktopFrame + bottomRight.getY32());
      polygon[1].set(cursorXDesktopFrame + bottomLeft.getX32(), cursorYDesktopFrame + bottomLeft.getY32());
      polygon[2].set(cursorXDesktopFrame + topLeft.getX32(), cursorYDesktopFrame + topLeft.getY32());
      polygon[3].set(cursorXDesktopFrame + toeLeft.getX32(), cursorYDesktopFrame + toeLeft.getY32());
      polygon[4].set(cursorXDesktopFrame + toeRight.getX32(), cursorYDesktopFrame + toeRight.getY32());
      polygon[5].set(cursorXDesktopFrame + topRight.getX32(), cursorYDesktopFrame + topRight.getY32());

      ImGui.getWindowDrawList().addConvexPolyFilled(polygon, 6, backgroundColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + (itemWidth * 1.2f));

      ImGui.newLine();
   }
}
