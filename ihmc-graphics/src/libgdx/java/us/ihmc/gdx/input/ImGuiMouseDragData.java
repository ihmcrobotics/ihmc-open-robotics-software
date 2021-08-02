package us.ihmc.gdx.input;

import imgui.internal.ImGui;

public class ImGuiMouseDragData
{
   private final int button;
   private boolean dragging = false;
   private float dragBucketX;
   private float dragBucketY;
   private float mouseDraggedX = 0.0f;
   private float mouseDraggedY = 0.0f;

   public ImGuiMouseDragData(int button)
   {
      this.button = button;
   }

   public void update(boolean isWindowHovered)
   {
      boolean mouseDown = ImGui.getIO().getMouseDown(button);
      float mouseDragDeltaX = ImGui.getMouseDragDeltaX(button);
      float mouseDragDeltaY = ImGui.getMouseDragDeltaY(button);

      if (!mouseDown)
      {
         dragging = false;
      }
      else if (isWindowHovered && !dragging)
      {
         dragging = true;
         dragBucketX = 0.0f;
         dragBucketY = 0.0f;
      }
      if (dragging)
      {
         mouseDraggedX = mouseDragDeltaX - dragBucketX;
         mouseDraggedY = mouseDragDeltaY - dragBucketY;

         dragBucketX += mouseDraggedX;
         dragBucketY += mouseDraggedY;
      }
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
}
