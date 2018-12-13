package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;

class TranslationMovingReferenceFrame extends MovingReferenceFrame
{
   public final FrameVector3D originVector;

   public TranslationMovingReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, parentFrame.isZupFrame());

      originVector = new FrameVector3D(parentFrame);
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {

   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.setIdentity();
      transformToParent.setTranslation(originVector);
   }

   public void updateTranslation(FrameTuple3DReadOnly frameVector)
   {
      originVector.set(frameVector);
      this.update();
   }

   public void updateTranslation(Tuple3DReadOnly translation)
   {
      originVector.set(translation);
      this.update();
   }
}
