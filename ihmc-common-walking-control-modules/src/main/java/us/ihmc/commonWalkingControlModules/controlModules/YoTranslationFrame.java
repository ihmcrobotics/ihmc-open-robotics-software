package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class YoTranslationFrame extends ReferenceFrame
{
   private final YoFrameVector3D translationToParent;

   public YoTranslationFrame(String frameName, ReferenceFrame parentFrame, YoVariableRegistry registry)
   {
      super(frameName, parentFrame);

      translationToParent = new YoFrameVector3D(frameName, parentFrame, registry);
   }

   public void setTranslationToParent(FrameTuple3DReadOnly translationToParent)
   {
      this.translationToParent.set(translationToParent);
      update();
   }

   public void setToZero()
   {
      translationToParent.setToZero();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.setTranslationAndIdentityRotation(translationToParent);
   }
}
