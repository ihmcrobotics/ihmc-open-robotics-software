package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoTranslationFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -7104467007470188012L;

   private final Vector3d tempVector = new Vector3d();
   private final YoFrameVector translationToParent;

   public YoTranslationFrame(String frameName, ReferenceFrame parentFrame, YoVariableRegistry registry)
   {
      super(frameName, parentFrame);

      translationToParent = new YoFrameVector(frameName, parentFrame, registry);
   }

   public void setTranslationToParent(FrameTuple<?> translationToParent)
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
      translationToParent.get(tempVector);
      transformToParent.setTranslationAndIdentityRotation(tempVector);
   }
}
