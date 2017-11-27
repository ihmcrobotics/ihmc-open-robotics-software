package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * In addition of having a defined pose in space as a {@code ReferenceFrame}, a
 * {@code MovingReferenceFrame} holds onto its velocity relative to its parent allowing to compute
 * its velocity with respect to world or another {@code MovingReferenceFrame}.
 * <p>
 * The parent of a {@code MovingReferenceFrame} can either be another {@code MovingReferenceFrame}
 * or a stationary {@code ReferenceFrame}, i.e. not moving with respect to its root frame.
 * </p>
 *
 */
public abstract class MovingReferenceFrame extends ReferenceFrame
{
   /**
    * Dirty bit used to mark {@link #twistOfFrame} as out-of-date or up-to-date and allow to save
    * some computation. The twist is marked as out-of-date upon calling {@link #update()}.
    */
   private boolean isTwistOfFrameUpToDate = false;
   /**
    * This is the data that needs to be provided from the user side.
    */
   private final Twist twistRelativeToParent;
   /**
    * This the computed twist of this frame relative to the root moving frame, i.e. the ancestor
    * frame that was created with a stationary {@code ReferenceFrame} as a parent.
    */
   private final Twist twistOfFrame = new Twist();

   /**
    * The parent moving reference frame.
    * <p>
    * It is {@code null} when this frame's parent is a stationary {@code ReferenceFrame}.
    * </p>
    */
   private final MovingReferenceFrame parentMovingFrame;

   /**
    * Whether the twist of this frame relative to its parent is equal to zero.
    */
   private final boolean isFixedInParent;

   /**
    * Constructs a {@code MovingReferenceFrame} that has a 'zero-velocity' with respect to its
    * parent.
    * <p>
    * The {@code transformToParent} should describe the pose of the new frame expressed in its
    * parent frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent of the new frame. It has to be either another
    *           {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame}, i.e. not moving
    *           with respect to its root frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *           frame to its parent frame. Not modified.
    * @return the new moving reference frame.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor
    *            a stationary {@code ReferenceFrame}.
    */
   public static MovingReferenceFrame constructFrameFixedInParent(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      boolean isZUpFrame = parentFrame.isZupFrame() && transformToParent.isRotation2D();
      boolean isFixedInParent = true;
      MovingReferenceFrame newFrame = new MovingReferenceFrame(frameName, parentFrame, transformToParent, isZUpFrame, isFixedInParent)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }

         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
         }
      };

      return newFrame;
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame}.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    * <p>
    * This frame is not expected to have its z-axis aligned at all time with the z-axis of the root
    * frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent of the new frame. It has to be either another
    *           {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame}, i.e. not moving
    *           with respect to its root frame.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor
    *            a stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      this(frameName, parentFrame, null, false, false);
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame} .
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent of the new frame. It has to be either another
    *           {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame}, i.e. not moving
    *           with respect to its root frame.
    * @param isZupFrame refers to whether this new frame has its z-axis aligned with the root frame
    *           at all time or not.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor
    *            a stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isZUpFrame)
   {
      this(frameName, parentFrame, new RigidBodyTransform(), isZUpFrame, false);
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame} and
    * initializes the transform to its parent.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    * <p>
    * This frame is not expected to have its z-axis aligned at all time with the z-axis of the root
    * frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent of the new frame. It has to be either another
    *           {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame}, i.e. not moving
    *           with respect to its root frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *           frame to its parent frame. Not modified.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor
    *            a stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      this(frameName, parentFrame, transformToParent, false, false);
   }

   /**
    * Creates a new moving reference frame as a child of the given {@code parentFrame} and
    * initializes the transform to its parent.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose and velocity with respect to the {@code parentFrame} can be modified at runtime by
    * changing the transform and twist in the methods
    * {@link #updateTransformToParent(RigidBodyTransform)} and
    * {@link #updateTwistRelativeToParent(Twist)} when overriding them.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent of the new frame. It has to be either another
    *           {@code MovingReferenceFrame} or a stationary {@code ReferenceFrame}, i.e. not moving
    *           with respect to its root frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *           frame to its parent frame. Not modified.
    * @param isZupFrame refers to whether this new frame has its z-axis aligned with the root frame
    *           at all time or not.
    * @throws ScrewTheoryException if {@code parentFrame} is not a {@code MovingReferenceFrame} nor
    *            a stationary {@code ReferenceFrame}.
    */
   public MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isZUpFrame)
   {
      this(frameName, parentFrame, transformToParent, isZUpFrame, false);
   }

   private MovingReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isZUpFrame, boolean isFixedInParent)
   {
      super(frameName, parentFrame, transformToParent, parentFrame.isAStationaryFrame() && isFixedInParent, isZUpFrame);

      this.isFixedInParent = isFixedInParent;

      if (parentFrame instanceof MovingReferenceFrame)
      {
         parentMovingFrame = (MovingReferenceFrame) parentFrame;
      }
      else
      {
         parentMovingFrame = null;

         if (!parentFrame.isAStationaryFrame())
            throw unhandledReferenceFrameTypeException(parentFrame);
      }

      if (isFixedInParent)
         twistRelativeToParent = null;
      else
         twistRelativeToParent = new Twist(this, parentFrame, this);
   }

   /**
    * In addition to performing {@code ReferenceFrame.update()}, it also marks the twist of this
    * reference frame as out-of-date such that it will be updated next time it is needed.
    * <p>
    * From {@code ReferenceFrame}:<br>
    * {@inheritDoc}
    * </p>
    * 
    * @throws ReferenceFrameMismatchException if the twist set in
    *            {@link #updateTwistRelativeToParent(Twist)} is not expressed with the proper
    *            frames, see {@link #updateTwistRelativeToParent(Twist)}.
    */
   @Override
   public void update()
   {
      super.update();

      if (!isFixedInParent)
      {
         updateTwistRelativeToParent(twistRelativeToParent);
         twistRelativeToParent.checkReferenceFramesMatch(this, parentFrame, this);
      }

      isTwistOfFrameUpToDate = false;
   }

   /**
    * Override this method to define what is the velocity of this moving reference frame relative to
    * its parent frame over time by setting the argument {@code twistRelativeToParentToPack}.
    * <p>
    * The frames of the {@code twistRelativeToParentToPack} should be as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code this}.
    * <li>{@code baseFrame} is {@code parentFrame}.
    * <li>{@code expressedInFrame} is {@code this}.
    * </ul>
    * </p>
    * 
    * @param twistRelativeToParentToPack the transform to updated according to how this reference
    *           frame should now positioned with respect to its parent frame. Modified.
    */
   protected abstract void updateTwistRelativeToParent(Twist twistRelativeToParentToPack);

   private void updateTwistOfFrame()
   {
      if (isTwistOfFrameUpToDateRecursive())
         return;

      if (parentMovingFrame == null)
      {
         if (isFixedInParent)
            twistOfFrame.setToZero(this, parentFrame, this);
         else
            twistOfFrame.set(twistRelativeToParent);
      }
      else
      {
         Twist parentTwist = parentMovingFrame.getTwistOfFrame();
         twistOfFrame.set(parentTwist);
         twistOfFrame.changeFrame(this);

         if (isFixedInParent)
            twistOfFrame.changeBodyFrameNoRelativeTwist(this);
         else
            twistOfFrame.add(twistRelativeToParent);
      }

      isTwistOfFrameUpToDate = true;
   }

   private boolean isTwistOfFrameUpToDateRecursive()
   {
      return isTwistOfFrameUpToDate && (parentMovingFrame == null || parentMovingFrame.isTwistOfFrameUpToDateRecursive());
   }

   /**
    * Gets the twist of this frame with respect the closest stationary frame, i.e. not moving with
    * respect to its root frame.
    * <p>
    * The returned twist the twist of {@code this} with respect to (usually)
    * {@link ReferenceFrame#getWorldFrame()} and expressed in {@code this}.
    * </p>
    * 
    * @return the absolute velocity of this frame. The returned object should not be modified.
    */
   public Twist getTwistOfFrame()
   {
      updateTwistOfFrame();
      return twistOfFrame;
   }

   /**
    * Packs the twist of this frame with respect the closest stationary frame, i.e. not moving with
    * respect to its root frame.
    * <p>
    * The returned twist the twist of {@code this} with respect to (usually)
    * {@link ReferenceFrame#getWorldFrame()} and expressed in {@code this}.
    * </p>
    * 
    * @param twistToPack the twist in which the absolute velocity of this frame is stored. Modified.
    */
   public void getTwistOfFrame(Twist twistToPack)
   {
      twistToPack.set(getTwistOfFrame());
   }

   /**
    * Computes the twist of this frame relative to the given {@code base}.
    * <p>
    * The reference frames of the resulting {@code Twist} are as follows:
    * <ul>
    * <li>{@code bodyFrame} is {@code this}.
    * <li>{@code baseFrame} is {@code base}.
    * <li>{@code expressedInFrame} is {@code this}.
    * </ul>
    * </p>
    * 
    * @param base the frame with respect to which the twist is to be computed.
    * @param relativeTwistToPack the twist of {@code this} with respect to {@code base}. Modified.
    */
   public void getTwistRelativeToOther(ReferenceFrame base, Twist relativeTwistToPack)
   {
      if (base.isAStationaryFrame())
      {
         getTwistOfFrame(relativeTwistToPack);
         relativeTwistToPack.changeBaseFrameNoRelativeTwist(base);
      }
      else if (base instanceof MovingReferenceFrame)
      {
         ((MovingReferenceFrame) base).getTwistOfFrame(relativeTwistToPack);
         relativeTwistToPack.changeFrame(this);
         relativeTwistToPack.sub(getTwistOfFrame());
         relativeTwistToPack.invert();
      }
      else
      {
         throw unhandledReferenceFrameTypeException(base);
      }
   }

   /**
    * Returns the parent moving frame of this moving reference frame.
    * <p>
    * Note that it is {@code null} when this moving frame is the child of a stationary
    * {@code ReferenceFrame}, i.e. not moving with respect to the root frame.
    * </p>
    *
    * @return the parent moving frame of this moving reference frame.
    */
   public MovingReferenceFrame getMovingParent()
   {
      return parentMovingFrame;
   }

   private static ScrewTheoryException unhandledReferenceFrameTypeException(ReferenceFrame referenceFrame)
   {
      return new ScrewTheoryException("The reference frame type: " + referenceFrame.getClass().getSimpleName()
            + " is currently not handled. Reference frame name: " + referenceFrame.getName());
   }
}
