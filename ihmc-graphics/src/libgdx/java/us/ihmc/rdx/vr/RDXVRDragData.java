package us.ihmc.rdx.vr;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import java.util.function.BooleanSupplier;

public class RDXVRDragData
{
   public static final double DRAG_TRANSLATION_TOLERANCE = 0.02;
   public static final double DRAG_ROTATION_TOLERANCE = Math.toRadians(3.0);

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
   private final MutableReferenceFrame dragReferenceFrame;
   /**
    * Used for dragging from a distance on the XY plane while the controller's
    * respective roll controls the yaw.
    */
   private final RDXVRPickPlaneYawCalculator pickPlaneYawCalculator = new RDXVRPickPlaneYawCalculator();
   private final MutableReferenceFrame zUpDragParentFrame;
   private final FramePose3D zUpDragPose = new FramePose3D();
   /**
    * Used for detecting clicks, which are most robustly a press and release
    * without a significant drag transform.
    */
   private final RigidBodyTransform dragStartToWorld = new RigidBodyTransform();
   private boolean clickValid = false;

   public RDXVRDragData(BooleanSupplier isButtonDown, ReferenceFrame controllerPickFrame)
   {
      this.isButtonDown = isButtonDown;
      this.controllerPickFrame = controllerPickFrame;
      dragReferenceFrame = new MutableReferenceFrame(controllerPickFrame);
      zUpDragParentFrame = new MutableReferenceFrame();
   }

   public void update()
   {
      boolean buttonUp = !isButtonDown.getAsBoolean();

      if (buttonUp)
      {
         // We want to invalidate the click, but leaving one frame
         // where dragging is false first.
         if (!dragging)
            clickValid = false;

         dragging = false;
         dragJustStarted = false;
      }
      else // Button down. We are now dragging.
      {
         dragJustStarted = !dragging;
         dragging = true;
         dragStartToWorld.set(controllerPickFrame.getTransformToRoot());
         clickValid = true;
      }

      if (buttonUp || dragJustStarted)
      {
         // We really want to make sure this is null when nothing is being dragged
         objectBeingDragged = null;
      }

      if (dragging)
      {
         double translationError = dragStartToWorld.getTranslation().differenceNorm(controllerPickFrame.getTransformToRoot().getTranslation());
         double rotationError = dragStartToWorld.getRotation().distance(controllerPickFrame.getTransformToRoot().getRotation(), true);
         clickValid &= translationError <= DRAG_TRANSLATION_TOLERANCE;
         clickValid &= rotationError <= DRAG_ROTATION_TOLERANCE;
      }
   }

   public boolean isDraggingSomething()
   {
      return dragging && objectBeingDragged != null;
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

   /**
    * Call this when starting to "grip" drag something, controlling
    * the whole 6 DoF pose.
    * @param interactableFrame The frame of the origin of the thing being grabbed.
    */
   public void setInteractableFrameOnDragStart(ReferenceFrame interactableFrame)
   {
      interactableFrame.getTransformToDesiredFrame(dragReferenceFrame.getTransformToParent(), controllerPickFrame);
      dragReferenceFrame.getReferenceFrame().update();
   }

   /**
    * Call this when beginning to drag something along the XY plane,
    * controlling the XY translation and yaw simultaneously.
    * @param interactableFrame The frame of the origin of the thing being grabbed.
    */
   public void setZUpDragStart(ReferenceFrame interactableFrame)
   {
      updateZUpDrag(interactableFrame);

      zUpDragParentFrame.setParentFrame(pickPlaneYawCalculator.getYawReferenceFrame());
      interactableFrame.getTransformToDesiredFrame(zUpDragParentFrame.getTransformToParent(), pickPlaneYawCalculator.getYawReferenceFrame());
      // This is a rotation only thing
      zUpDragParentFrame.getTransformToParent().getTranslation().setToZero();
      zUpDragParentFrame.getReferenceFrame().update();
   }

   /**
    * Call this once per frame while dragging something on the XY plane.
    * Doesn't need to be called on the drag start frame.
    * @param interactableFrame The frame of the origin of the thing being grabbed.
    */
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

   /* package-private */ boolean isClickValid()
   {
      return clickValid;
   }
}