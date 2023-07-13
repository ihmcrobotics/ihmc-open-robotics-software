package us.ihmc.rdx.vr;

import java.util.function.BooleanSupplier;

public class RDXVRDragData
{
   private final BooleanSupplier isButtonDown;
   private boolean dragging = false;
   private boolean dragJustStarted = false;
   private Object objectBeingDragged = null;

   public RDXVRDragData(BooleanSupplier isButtonDown)
   {
      this.isButtonDown = isButtonDown;
   }

   public void update()
   {
      boolean buttonUp = !isButtonDown.getAsBoolean();

      if (buttonUp)
      {
         dragging = false;
         dragJustStarted = false;
      }
      else // Button down. We are now dragging.
      {
         dragJustStarted = !dragging;
         dragging = true;
      }

      if (buttonUp || dragJustStarted)
      {
         // We really want to make sure this is null when nothing is being dragged
         objectBeingDragged = null;
      }
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
