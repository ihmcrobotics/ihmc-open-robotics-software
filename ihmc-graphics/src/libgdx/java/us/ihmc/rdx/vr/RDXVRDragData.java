package us.ihmc.rdx.vr;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

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
   private final ModifiableReferenceFrame dragReferenceFrame;
   /**
    * Used for dragging from a distance on the XY plane while the controller's
    * respective roll controls the yaw.
    */
   private final RDXVRPickPlaneYawCalculator pickPlaneYawCalculator = new RDXVRPickPlaneYawCalculator();
   private final ModifiableReferenceFrame zUpDragParentFrame;
   private final FramePose3D zUpDragPose = new FramePose3D();

   public RDXVRDragData(BooleanSupplier isButtonDown, ReferenceFrame controllerPickFrame)
   {
      this.isButtonDown = isButtonDown;
      this.controllerPickFrame = controllerPickFrame;
      dragReferenceFrame = new ModifiableReferenceFrame(controllerPickFrame);
      zUpDragParentFrame = new ModifiableReferenceFrame();
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
    * <p>
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
      interactableFrame.getTransformToDesiredFrame(dragReferenceFrame.getTransformToParent(), controllerPickFrame);
      dragReferenceFrame.getReferenceFrame().update();
   }

   public void setZUpDragStart(ReferenceFrame interactableFrame)
   {
      updateZUpDrag(interactableFrame);

      zUpDragParentFrame.changeParentFrame(pickPlaneYawCalculator.getYawReferenceFrame());
      interactableFrame.getTransformToDesiredFrame(zUpDragParentFrame.getTransformToParent(), pickPlaneYawCalculator.getYawReferenceFrame());
      // This is a rotation only thing
      zUpDragParentFrame.getTransformToParent().getTranslation().setToZero();
      zUpDragParentFrame.getReferenceFrame().update();
   }

   public void updateZUpDrag(ReferenceFrame interactableFrame)
   {
      pickPlaneYawCalculator.calculate(controllerPickFrame, interactableFrame);
   }

   public ReferenceFrame getDragFrame()
   {
      return dragReferenceFrame.getReferenceFrame();
   }

   public ReferenceFrame getZUpDragFrame()
   {
      return zUpDragParentFrame.getReferenceFrame();
   }

   public FramePose3D getZUpDragPose()
   {
      zUpDragPose.setToZero(getZUpDragFrame());
      zUpDragPose.changeFrame(ReferenceFrame.getWorldFrame());
      return zUpDragPose;
   }
}