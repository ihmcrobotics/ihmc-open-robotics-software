package us.ihmc.rdx.vr;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.util.function.BooleanSupplier;

public class RDXVRDragData
{
   private final BooleanSupplier isButtonDown;
   private boolean dragging = false;
   private boolean dragJustStarted = false;
   private Object objectBeingDragged = null;
   private final ReferenceFrame controllerPickFrame;
   /**
    * Drag reference frame is a child of the controller pick frame.
    * That drag reference frame's transform to the controller pick frame
    * stays static during the drag.
    * The drag reference frame is meant to be at some useful part of the thing
    * being dragged. Most of the time it's the origin of the thing being dragged.
    * This is so we can directly pack that interactable thing's new transform to
    * world from this reference frame's transform to world.
    * Because the drag frame's parent, the controller, is moving, the drag frame will too
    * (without even updating it), so we can safely grab the transform to world
    * and set the interactable's pose to it.
    */
   private final ReferenceFrame dragReferenceFrame;
   private final RigidBodyTransform dragToControllerPickTransform = new RigidBodyTransform();

   public RDXVRDragData(BooleanSupplier isButtonDown, ReferenceFrame controllerPickFrame)
   {
      this.isButtonDown = isButtonDown;
      this.controllerPickFrame = controllerPickFrame;
      dragReferenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(controllerPickFrame, dragToControllerPickTransform);
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

   public Object getObjectBeingDragged()
   {
      return objectBeingDragged;
   }

   public void setInteractableFrameOnDragStart(ReferenceFrame interactableFrame)
   {
      interactableFrame.getTransformToDesiredFrame(dragToControllerPickTransform, controllerPickFrame);
      dragReferenceFrame.update();
   }

   public ReferenceFrame getDragFrame()
   {
      return dragReferenceFrame;
   }
}