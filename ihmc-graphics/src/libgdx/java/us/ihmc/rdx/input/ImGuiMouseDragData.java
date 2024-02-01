package us.ihmc.rdx.input;

import imgui.ImGui;

/**
 * ImGui drag doesn't start until the mouse has dragged a little. This might be system dependent.
 * We want the drags to start immidiately before even a 1 px movement occurs. This allows
 * for maximum precision.
 *
 * So we intentionally do not use {@link ImGui#isMouseDragging}.
 */
public class ImGuiMouseDragData
{
   private final int button;
   private boolean dragging = false;
   private boolean dragJustStarted = false;
   private float lastMousePositionX;
   private float lastMousePositionY;
   private float mouseDraggedX = 0.0f;
   private float mouseDraggedY = 0.0f;
   private Object objectBeingDragged = null;

   public ImGuiMouseDragData(int button)
   {
      this.button = button;
   }

   public void update()
   {
      boolean mouseUp = !ImGui.isMouseDown(button);
      float mousePositionX = ImGui.getMousePosX();
      float mousePositionY = ImGui.getMousePosY();
      float mouseDragDeltaX = mousePositionX - lastMousePositionX;
      float mouseDragDeltaY = mousePositionY - lastMousePositionY;

      if (mouseUp)
      {
         dragging = false;
         dragJustStarted = false;
      }
      else // Mouse down. We are now dragging.
      {
         dragJustStarted = !dragging;
         dragging = true;

         mouseDraggedX = mouseDragDeltaX;
         mouseDraggedY = mouseDragDeltaY;
      }

      if (mouseUp || dragJustStarted)
      {
         // We really want to make sure this is null when nothing is being dragged
         objectBeingDragged = null;
      }

      lastMousePositionX = mousePositionX;
      lastMousePositionY = mousePositionY;
   }

   public boolean isDragging()
   {
      return dragging;
   }

   public boolean isBeingDragged(Object objectInQuestion)
   {
      return objectBeingDragged == objectInQuestion && dragging;
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

   /**
    * Since this class is global to the whole 3D panel, we use
    * this to keep track of what is being dragged. Each thing
    * that has drag support wants to know if it is the thing in
    * the foreground.
    *
    * Set this after a call to {@link #getDragJustStarted()} returns true.
    *
    * @param objectBeingDragged Any user object, just for checking equals.
    */
   public void setObjectBeingDragged(Object objectBeingDragged)
   {
      this.objectBeingDragged = objectBeingDragged;
   }
}
