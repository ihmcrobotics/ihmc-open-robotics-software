package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * This defines a mutable {@link ReferenceFrame}, where the pose of this frame relative to the parent {@link ReferenceFrame}, which is accessible as
 * {@link #getParent()} can be updated via a variety of methods, but exists only in the 2D plane.
 */
public class Pose2DReferenceFrame extends ReferenceFrame
{
   /**
    * The 2D pose of this frame relative to its parent.
    */
   private final FramePose2D originPose;

   /**
    * <p>
    * Creates a new pose 2D reference frame as a child of the given {@param parentFrame} and initializes the transform to its parent frame.
    * </p>
    * <p>
    * This new reference frame is defined in the {@param parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the parent frame can be modified at runtime by changing the transform in the methods outlined in this class, including setting
    * the pose {@link #setPoseAndUpdate(FramePose2DReadOnly)}, setting the pose from a reference frame {@link #setPoseAndUpdate(Pose2DReferenceFrame)}, setting
    * the translation {@link #setPositionAndUpdate(FramePoint2DReadOnly)}, or setting the orientation
    * {@link #setOrientationAndUpdate(FrameOrientation2DReadOnly)}.
    * </p>
    *
    * @param frameName   the name of the new frame.
    * @param parentFrame the parent of the frame. It has to extend a {@link ReferenceFrame}.
    */
   public Pose2DReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);

      originPose = new FramePose2D(parentFrame);
   }

   public Pose2DReferenceFrame(String frameName, FramePose2DReadOnly poseInParentFrame)
   {
      this(frameName, poseInParentFrame.getReferenceFrame());
      setPoseAndUpdate(poseInParentFrame);
   }

   /** {@inheritDoc} */
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      originPose.get(transformToParent);
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in tuple {@param pose} must be defined as
    * {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setPoseAndUpdate(Pose2DReadOnly)}.
    *
    * @param pose pose of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(FramePose2DReadOnly pose)
   {
      pose.checkReferenceFrameMatch(getParent());
      setPoseAndUpdate((Pose2DReadOnly) pose);
   }

   /**
    * Sets the position of this reference frame relative to its parent. The frame of the passed in tuple {@param position} must be defined
    * as {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setPositionAndUpdate(Point2DReadOnly)}.
    *
    * @param position position of this frame relative to parent to set.
    */
   public void setPositionAndUpdate(FramePoint2DReadOnly position)
   {
      position.checkReferenceFrameMatch(getParent());
      setPositionAndUpdate((Point2DReadOnly) position);
   }

   /**
    * Sets the orientation of this reference frame relative to its parent. The frame of the passed in tuple {@param orientation} must be defined
    * as {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setOrientationAndUpdate(Orientation2DReadOnly)}.
    *
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setOrientationAndUpdate(FrameOrientation2DReadOnly orientation)
   {
      orientation.checkReferenceFrameMatch(getParent());
      setOrientationAndUpdate((Orientation2DReadOnly) orientation);
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in tuples {@param position} and {@param orientation} must be defined
    * as {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setPoseAndUpdate(Pose2DReadOnly)} or {@link #setPoseAndUpdate(Point2DReadOnly, Orientation2DReadOnly)}.
    *
    * @param position    position of this frame relative to parent to set.
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(FramePoint2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      position.checkReferenceFrameMatch(getParent());
      orientation.checkReferenceFrameMatch(getParent());
      setPoseAndUpdate((Point2DReadOnly) position, (Orientation2DReadOnly) orientation);
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in {@param pose} must be defined as
    * {@link #getParent()}. This method does not perform a frame check. For frame safe operations, see {@link #setPoseAndUpdate(FramePose2DReadOnly)}.
    *
    * @param pose pose of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(Pose2DReadOnly pose)
   {
      setPoseAndUpdate(pose.getPosition(), pose.getOrientation());
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in {@param position} and {@param orientation} must be defined as
    * {@link #getParent()}. This method does not perform a frame check. For frame safe operations, see
    * {@link #setPoseAndUpdate(FramePoint2DReadOnly, FrameOrientation2DReadOnly)}.
    *
    * @param position    position of this frame relative to parent to set.
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(Point2DReadOnly position, Orientation2DReadOnly orientation)
   {
      setPoseAndUpdate(position, orientation.getYaw());
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in {@param pose} must be defined as
    * {@link #getParent()}. This method does not perform a frame check. For frame safe operations, see
    * {@link #setPoseAndUpdate(FramePoint2DReadOnly, FrameOrientation2DReadOnly)}.
    *
    * @param position    position of this frame relative to parent to set.
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(Point2DReadOnly position, double orientation)
   {
      originPose.getPosition().set(position);
      originPose.getOrientation().setYaw(orientation);
      this.update();
   }

   /**
    * Sets the position of this reference frame relative to its parent. This does not perform a frame check. For frame safe operations, please use
    * {@link #setPositionAndUpdate(FramePoint2DReadOnly)}.
    *
    * @param position position of this frame relative to parent to set.
    */
   public void setPositionAndUpdate(Point2DReadOnly position)
   {
      originPose.getPosition().set(position);
      this.update();
   }

   /**
    * Sets the orientation of this reference frame relative to its parent. This does not perform a frame check. For frame safe operations, please use
    * {@link #setOrientationAndUpdate(FrameOrientation2DReadOnly)}.
    *
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setOrientationAndUpdate(Orientation2DReadOnly orientation)
   {
      originPose.getOrientation().set(orientation);
      this.update();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String toString()
   {
      return super.toString() + ", originPose = " + originPose;
   }
}
