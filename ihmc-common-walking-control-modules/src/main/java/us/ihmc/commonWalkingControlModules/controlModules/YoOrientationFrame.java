package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoOrientationFrame extends MovingReferenceFrame
{
   private final YoFrameQuaternion rotationToParent;

   /**
    * Creates a new reference frame with a mutable offset to its parent frame.
    * <br>
    * The offset to the parent frame is backed by a {@link YoFrameQuaternion} internally.
    *
    * @param frameName the name of this reference frame.
    * @param parentFrame the frame to which this is attached.
    * @param registry the registry to which the internal {@code YoVariable}s are registered.
    */
   public YoOrientationFrame(String frameName, ReferenceFrame parentFrame, YoRegistry registry)
   {
      super(frameName, parentFrame);

      rotationToParent = new YoFrameQuaternion(frameName, parentFrame, registry);
   }

   /**
    * Creates a new reference frame with a mutable offset to its parent frame.
    * <br>
    * The offset to the parent frame is stored in {@code rotationToParent}.
    *
    * @param frameName the name of this reference frame.
    * @param rotationToParent rotation offset to parent frame
    * @param parentFrame the frame to which this is attached.
    */
   public YoOrientationFrame(String frameName, YoFrameQuaternion rotationToParent, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);

      this.rotationToParent = rotationToParent;
   }

   /**
    * Updates the offset of this frame to the parent frame.
    *
    * @param rotationToParent the orientation of this frame origin expressed in its parent frame. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in {@code this.getParent()}.
    */
   public void setRotationToParent(FrameQuaternionReadOnly rotationToParent)
   {
      this.rotationToParent.set(rotationToParent);
      update();
   }

   /**
    * Resets the offset such that this frame is located and aligned with its parent frame.
    */
   public void setToZero()
   {
      rotationToParent.setToZero();
      update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.setRotationAndZeroTranslation(rotationToParent);
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      twistRelativeToParentToPack.setToZero(this, getParent(), this);
   }
}
