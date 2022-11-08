package us.ihmc.rdx.input;

import imgui.ImGui;

public class ImGuiMouseDragData
{
   private final int button;
   private boolean dragging = false;
   private boolean dragJustStarted = false;
   private float lastMousePositionX;
   private float lastMousePositionY;
   private float mouseDraggedX = 0.0f;
   private float mouseDraggedY = 0.0f;
   private Object objectBeingDragged;

   public ImGuiMouseDragData(int button)
   {
      this.button = button;
   }

   public void update()
   {
      boolean mouseDown = ImGui.isMouseDown(button);
      float mousePositionX = ImGui.getMousePosX();
      float mousePositionY = ImGui.getMousePosY();
      float mouseDragDeltaX = mousePositionX - lastMousePositionX;
      float mouseDragDeltaY = mousePositionY - lastMousePositionY;

      if (!mouseDown)
      {
         dragging = false;
         objectBeingDragged = null;
      }
      // ImGui drag doesn't start until the mouse has dragged a little; probably system dependent
      //  else if (ImGui.isMouseDragging(button))
      // It's probably better without that
      else
      {
         // We are now dragging

         dragJustStarted = !dragging;
         dragging = true;

         mouseDraggedX = mouseDragDeltaX;
         mouseDraggedY = mouseDragDeltaY;
      }

      lastMousePositionX = mousePositionX;
      lastMousePositionY = mousePositionY;
   }

   public boolean isDragging()
   {
      return dragging;
   }

   public boolean getDragJustStarted()
   {
      return dragJustStarted;
   }

   public float getMouseDraggedX()
   {
      return mouseDraggedX;
   }

   public float getMouseDraggedY()
   {
      return mouseDraggedY;
   }

   public void setObjectBeingDragged(Object objectBeingDragged)
   {
      this.objectBeingDragged = objectBeingDragged;
   }

   public Object getObjectBeingDragged()
   {
      return objectBeingDragged;
   }
}
