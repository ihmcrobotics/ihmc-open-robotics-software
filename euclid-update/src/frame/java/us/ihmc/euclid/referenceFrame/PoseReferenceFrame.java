package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * This defines a mutable {@link ReferenceFrame}, where the pose of this frame relative to the parent {@link ReferenceFrame}, which is accessible as
 * {@link #getParent()} can be updated via a variety of methods.
 */
public class PoseReferenceFrame extends ReferenceFrame
{
   /**
    * The pose of this frame relative to its parent.
    */
   private final FramePose3D originPose;

   /**
    * <p>
    * Creates a new pose reference frame as a child of the given {@param parentFrame} and initializes the transform to its parent frame.
    * </p>
    * <p>
    * This new reference frame is defined in the {@param parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the parent frame can be modified at runtime by changing the transform in the methods outlined in this class, including setting
    * the pose {@link #setPoseAndUpdate(FramePose3DReadOnly)}, setting the pose from a reference frame {@link #setPoseAndUpdate(PoseReferenceFrame)}, setting
    * the translation {@link #setPositionAndUpdate(FramePoint3DReadOnly)}, setting the orientation {@link #setOrientationAndUpdate(FrameOrientation3DReadOnly)},
    * or setting the transform {@link #setTransformAndUpdate(RigidBodyTransformReadOnly)}
    * </p>
    *
    * @param frameName   the name of the new frame.
    * @param parentFrame the parent of the frame. It has to extend a {@link ReferenceFrame}.
    */
   public PoseReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, parentFrame.isAStationaryFrame(), false);

      originPose = new FramePose3D(parentFrame);
   }

   /**
    * <p>
    * Creates a new pose reference frame with a defined pose relative to the parent frame, both of which are defined by {@param poseInParentFrame}.
    * </p>
    * <p>
    * Its pose with respect to the parent frame can be modified at runtime by changing the transform in the methods outlined in this class, including setting
    * the pose {@link #setPoseAndUpdate(FramePose3DReadOnly)}, setting the pose from a reference frame {@link #setPoseAndUpdate(PoseReferenceFrame)}, setting
    * the translation {@link #setPositionAndUpdate(FramePoint3DReadOnly)}, setting the orientation {@link #setOrientationAndUpdate(FrameOrientation3DReadOnly)},
    * or setting the transform {@link #setTransformAndUpdate(RigidBodyTransformReadOnly)}
    * </p>
    *
    * @param frameName         the name of the new frame.
    * @param poseInParentFrame pose of the frame, with the parent frame provided by {@param poseInParentFrame.getParent()}.
    */
   public PoseReferenceFrame(String frameName, FramePose3DReadOnly poseInParentFrame)
   {
      this(frameName, poseInParentFrame.getReferenceFrame());
      setPoseAndUpdate(poseInParentFrame);
   }

   /**
    * <p>
    * Creates a new pose reference frame as a child of the given {@param parentFrame} and initializes the transform to its parent frame.
    * </p>
    * <p>
    * This new reference frame is defined with a transform relative to the parent  {@param parentFrame} as transform {@param transformToParent}.
    * </p>
    * <p>
    * Its pose with respect to the parent frame can be modified at runtime by changing the transform in the methods outlined in this class, including setting
    * the pose {@link #setPoseAndUpdate(FramePose3DReadOnly)}, setting the pose from a reference frame {@link #setPoseAndUpdate(PoseReferenceFrame)}, setting
    * the translation {@link #setPositionAndUpdate(FramePoint3DReadOnly)}, setting the orientation {@link #setOrientationAndUpdate(FrameOrientation3DReadOnly)},
    * or setting the transform {@link #setTransformAndUpdate(RigidBodyTransformReadOnly)}
    * </p>
    *
    * @param frameName   the name of the new frame.
    * @param parentFrame the parent of the frame. It has to extend a {@link ReferenceFrame}.
    */
   public PoseReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      this(frameName, parentFrame);
      setTransformAndUpdate(transformToParent);
   }

   /**
    * Sets the pose of this reference frame relative to its parent to match that of {@param poseReferenceFrame}. The parent frame of the passed in frame
    * {@param poseReferenceFrame} must match this frame's. If it does not, this method will throw a {@link ReferenceFrameMismatchException}.
    *
    * @param poseReferenceFrame frame containing the pose to copy.
    */
   public void setPoseAndUpdate(PoseReferenceFrame poseReferenceFrame)
   {
      checkReferenceFrameMatch(poseReferenceFrame.getParent());
      setPoseAndUpdate(poseReferenceFrame.originPose);
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in tuple {@param pose} must be defined as
    * {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setPoseAndUpdate(Pose3DReadOnly)}
    *
    * @param pose pose of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(FramePose3DReadOnly pose)
   {
      checkReferenceFrameMatch(pose);
      setPoseAndUpdate((Pose3DReadOnly) pose);
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in tuples {@param position} and {@param orientation} must be defined
    * as {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setPoseAndUpdate(Pose3DReadOnly)} or {@link #setPoseAndUpdate(Point3DReadOnly, Orientation3DReadOnly)}.
    *
    * @param position    position of this frame relative to parent to set.
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      position.checkReferenceFrameMatch(getParent());
      orientation.checkReferenceFrameMatch(getParent());
      setPoseAndUpdate((Point3DReadOnly) position, (Orientation3DReadOnly) orientation);
   }

   /**
    * Sets the position of this reference frame relative to its parent. The frame of the passed in tuple {@param position} must be defined
    * as {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setPositionAndUpdate(Point3DReadOnly)}.
    *
    * @param position position of this frame relative to parent to set.
    */
   public void setPositionAndUpdate(FramePoint3DReadOnly position)
   {
      position.checkReferenceFrameMatch(getParent());
      setPositionAndUpdate((Point3DReadOnly) position);
   }

   /**
    * Sets the orientation of this reference frame relative to its parent. The frame of the passed in tuple {@param orientation} must be defined
    * as {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setOrientationAndUpdate(Orientation3DReadOnly)}.
    *
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setOrientationAndUpdate(FrameOrientation3DReadOnly orientation)
   {
      orientation.checkReferenceFrameMatch(getParent());
      setOrientationAndUpdate((Orientation3DReadOnly) orientation);
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in {@param pose} must be defined as
    * {@link #getParent()}. This method does not perform a frame check. For frame safe operations, see {@link #setPoseAndUpdate(FramePose3DReadOnly)}.
    *
    * @param pose pose of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(Pose3DReadOnly pose)
   {
      setPoseAndUpdate(pose.getPosition(), pose.getOrientation());
   }

   /**
    * Sets the pose of this reference frame relative to its parent. The frame of the passed in {@param position} and {@param orientation}  must be defined as
    * {@link #getParent()}. This method does not perform a frame check. For frame safe operations, see
    * {@link #setPoseAndUpdate(FramePoint3DReadOnly, FrameOrientation3DReadOnly)}.
    *
    * @param position    position of this frame relative to parent to set.
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setPoseAndUpdate(Point3DReadOnly position, Orientation3DReadOnly orientation)
   {
      originPose.set(position, orientation);
      this.update();
   }

   /**
    * Sets the transform of this reference frame relative to its parent to be {@param transform}.
    *
    * @param transformToParent transform relative to parent to set.
    */
   public void setTransformAndUpdate(RigidBodyTransformReadOnly transformToParent)
   {
      originPose.set(transformToParent);
      this.update();
   }

   /**
    * Sets the position of this reference frame relative to its parent. This does not perform a frame check. For frame safe operations, please use
    * {@link #setPositionAndUpdate(FramePoint3DReadOnly)}.
    *
    * @param position position of this frame relative to parent to set.
    */
   public void setPositionAndUpdate(Point3DReadOnly position)
   {
      setPositionAndUpdate(position.getX(), position.getY(), position.getZ());
   }

   /**
    * Sets the orientation of this reference frame relative to its parent. This does not perform a frame check. For frame safe operations, please use
    * {@link #setOrientationAndUpdate(FrameOrientation3DReadOnly)}.
    *
    * @param orientation orientation of this frame relative to parent to set.
    */
   public void setOrientationAndUpdate(Orientation3DReadOnly orientation)
   {
      originPose.getOrientation().set(orientation);
      this.update();
   }

   /**
    * Sets the position of this reference frame relative to its parent to the corresponding x, y, and z locations.
    *
    * @param x x position of this frame relative to parent to set.
    * @param y y position of this frame relative to parent to set.
    * @param z z position of this frame relative to parent to set.
    */
   public void setPositionAndUpdate(double x, double y, double z)
   {
      originPose.getPosition().set(x, y, z);
      this.update();
   }

   /**
    * Sets the quaternion coordinations of this reference frame relative to its parent to the corresponding qx, qy, qz, and qs locations.
    *
    * @param qx quaternion value qx of this frame relative to parent to set.
    * @param qy quaternion value qy of this frame relative to parent to set.
    * @param qz quaternion value qz of this frame relative to parent to set.
    * @param qs quaternion value qs of this frame relative to parent to set.
    */
   public void setOrientationAndUpdate(double qx, double qy, double qz, double qs)
   {
      originPose.getOrientation().set(qx, qy, qz, qs);
      update();
   }

   /**
    * Prepends a translation to this reference frame, relative to its parent, of a magnitude of x, y, and z.
    *
    * @param x x translation to prepend relative to the parent.
    * @param y y translation to prepend relative to the parent.
    * @param z z translation to prepend relative to the parent.
    */
   public void prependTranslationAndUpdate(double x, double y, double z)
   {
      originPose.prependTranslation(x, y, z);
      this.update();
   }

   /**
    * Returns the x translation of this frame relative to the parent frame.
    *
    * @return x translation
    */
   public double getTranslationX()
   {
      return getPose().getX();
   }

   /**
    * Returns the y translation of this frame relative to the parent frame.
    *
    * @return y translation
    */
   public double getTranslationY()
   {
      return getPose().getY();
   }

   /**
    * Returns the z translation of this frame relative to the parent frame.
    *
    * @return z translation
    */
   public double getTranslationZ()
   {
      return getPose().getZ();
   }

   /**
    * Returns the pose of this frame relative to the parent frame.
    *
    * @return pose of this frame relative to the parent.
    */
   public Pose3DReadOnly getPose()
   {
      return originPose;
   }

   /**
    * Returns the translation of this frame relative to the parent frame.
    *
    * @return translation of this frame relative to the parent.
    */
   public Point3DReadOnly getTranslation()
   {
      return getPose().getPosition();
   }

   /**
    * Returns the orientation of this frame relative to the parent frame.
    *
    * @return orientation of this frame relative to the parent.
    */
   public QuaternionReadOnly getOrientation()
   {
      return getPose().getOrientation();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParentToPack)
   {
      originPose.checkReferenceFrameMatch(getParent());
      transformToParentToPack.set(originPose);
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
