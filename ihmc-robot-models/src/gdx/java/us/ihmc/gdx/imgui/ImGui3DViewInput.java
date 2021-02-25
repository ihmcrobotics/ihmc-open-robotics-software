package us.ihmc.gdx.imgui;

import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

/**
 * This class is just to do the non-trivial or high computation stuff.
 * Simple things like ImGui.isKeyDown('W') should still just use the
 * ImGui API for now.
 */
public class ImGui3DViewInput
{
   private boolean dragging = false;
   private float dragBucketX;
   private float dragBucketY;

   private float mouseDraggedX = 0.0f;
   private float mouseDraggedY = 0.0f;
   private float mousePosX = 0.0f;
   private float mousePosY = 0.0f;
   private boolean isWindowHovered;
   private float mouseWheelDelta;

   public void compute(GDXImGuiBasedUI baseUI)
   {
      boolean leftMouseDown = ImGui.getIO().getMouseDown(ImGuiMouseButton.Left);
      float mouseDragDeltaX = ImGui.getMouseDragDeltaX();
      float mouseDragDeltaY = ImGui.getMouseDragDeltaY();
      isWindowHovered = ImGui.isWindowHovered();
      mousePosX = (int) ImGui.getMousePosX() - (int) ImGui.getWindowPosX();
      mousePosY = (int) ImGui.getMousePosY() - (int) ImGui.getWindowPosY() - (int) ImGuiTools.TAB_BAR_HEIGHT;
      mouseWheelDelta = -ImGui.getIO().getMouseWheel();

      if (!leftMouseDown)
      {
         dragging = false;
      }
      else if (isWindowHovered && (mouseDragDeltaX != 0.0f || mouseDragDeltaY != 0.0f) && !dragging)
      {
         dragging = true;
         dragBucketX = 0.0f;
         dragBucketY = 0.0f;
      }
      if (dragging)
      {
         mouseDraggedX = mouseDragDeltaX - dragBucketX;
         mouseDraggedY = mouseDragDeltaY - dragBucketY;

         dragBucketX += mouseDragDeltaX - dragBucketX;
         dragBucketY += mouseDragDeltaY - dragBucketY;
      }
   }

   public boolean isWindowHovered()
   {
      return isWindowHovered;
   }

   public boolean isDragging()
   {
      return dragging;
   }

   public float getMouseDraggedX()
   {
      return mouseDraggedX;
   }

   public float getMouseDraggedY()
   {
      return mouseDraggedY;
   }

   public float getMousePosX()
   {
      return mousePosX;
   }

   public float getMousePosY()
   {
      return mousePosY;
   }

   public float getMouseWheelDelta()
   {
      return mouseWheelDelta;
   }
}
