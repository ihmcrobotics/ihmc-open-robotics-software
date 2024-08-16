package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoTranslationFrame extends MovingReferenceFrame
{
   private final YoFrameVector3D translationToParent;

   /**
    * Creates a new reference frame with a mutable offset to its parent frame.
    * <p>
    * The offset to the parent frame is backed by a {@link YoFrameVector3D} and a
    * {@link YoFrameQuaternion} internally.
    * </p>
    *
    * @param frameName the name of this reference frame.
    * @param parentFrame the frame to which this is attached.
    * @param registry the registry to which the internal {@code YoVariable}s are registered.
    */
   public YoTranslationFrame(String frameName, ReferenceFrame parentFrame, YoRegistry registry)
   {
      super(frameName, parentFrame);

      translationToParent = new YoFrameVector3D(frameName, parentFrame, registry);
   }

   /**
    * Creates a new reference frame with a mutable offset to its parent frame.
    * <br>
    * The offset to the parent frame is stored in {@code translationToParent}.
    *
    * @param frameName the name of this reference frame.
    * @param translationToParent translation offset to parent frame
    * @param parentFrame the frame to which this is attached.
    */
   public YoTranslationFrame(String frameName, YoFrameVector3D translationToParent, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);

      this.translationToParent = translationToParent;
   }

   /**
    * Updates the offset of this frame to the parent frame.
    *
    * @param translationToParent the position of this frame origin expressed in its parent frame. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in {@code this.getParent()}.
    */
   public void setTranslationToParent(FrameTuple3DReadOnly translationToParent)
   {
      this.translationToParent.set(translationToParent);
      update();
   }

   /**
    * Resets the offset such that this frame is located and aligned with its parent frame.
    */
   public void setToZero()
   {
      translationToParent.setToZero();
      update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.setTranslationAndIdentityRotation(translationToParent);
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      twistRelativeToParentToPack.setToZero(this, getParent(), this);
   }
}
