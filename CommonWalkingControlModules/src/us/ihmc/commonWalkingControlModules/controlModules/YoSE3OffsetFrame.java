package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;

public class YoSE3OffsetFrame extends MovingReferenceFrame
{
   private static final long serialVersionUID = 2800529580025439076L;
   private final Vector3D tempVector = new Vector3D();
   private final Quaternion tempQuaternion = new Quaternion();
   private final YoFrameVector translationToParent;
   private final YoFrameQuaternion rotationToParent;

   /**
    * Creates a new reference frame with a mutable offset to its parent frame.
    * <p>
    * The offset to the parent frame is backed by a {@link YoFrameVector} and a
    * {@link YoFrameQuaternion} internally.
    * </p>
    * 
    * @param frameName the name of this reference frame.
    * @param parentFrame the frame to which this is attached.
    * @param registry the registry to which the internal {@code YoVariable}s are registered.
    */
   public YoSE3OffsetFrame(String frameName, ReferenceFrame parentFrame, YoVariableRegistry registry)
   {
      super(frameName, parentFrame);

      translationToParent = new YoFrameVector(frameName, parentFrame, registry);
      rotationToParent = new YoFrameQuaternion(frameName, parentFrame, registry);
   }

   /**
    * Sets the offset to be a pure translation.
    * <p>
    * This method resets the orientation offset.
    * </p>
    * 
    * @param translationToParent the position of this frame origin expressed in its parent frame.
    *           Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@code this.getParent()}.
    */
   public void setOffsetToParentToTranslationOnly(FrameTuple<?, ?> translationToParent)
   {
      this.translationToParent.set(translationToParent);
      this.rotationToParent.setToZero();
      update();
   }

   /**
    * Sets the offset of this frame to its parent.
    * 
    * @param translationToParent the position of this frame origin expressed in its parent frame.
    *           Not modified.
    * @param rotationToParent the orientation of this frame expressed in its parent frame. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if any of the two arguments is not expressed in
    *            {@code this.getParent()}.
    */
   public void setOffsetToParent(FrameTuple<?, ?> translationToParent, FrameOrientation rotationToParent)
   {
      this.translationToParent.set(translationToParent);
      this.rotationToParent.set(rotationToParent);
      update();
   }

   /**
    * Resets the offset such that this frame is located and aligned with its parent frame.
    */
   public void setToZero()
   {
      translationToParent.setToZero();
      rotationToParent.setToZero();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      translationToParent.get(tempVector);
      rotationToParent.get(tempQuaternion);
      transformToParent.set(tempQuaternion, tempVector);
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      twistRelativeToParentToPack.setToZero(this, parentFrame, this);
   }
}
