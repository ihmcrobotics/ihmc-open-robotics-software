package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoSE3OffsetFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 2800529580025439076L;
   private final Vector3D tempVector = new Vector3D();
   private final Quaternion tempQuaternion = new Quaternion();
   private final YoFrameVector translationToParent;
   private final YoFrameQuaternion rotationToParent;

   public YoSE3OffsetFrame(String frameName, ReferenceFrame parentFrame, YoVariableRegistry registry)
   {
      super(frameName, parentFrame);

      translationToParent = new YoFrameVector(frameName, parentFrame, registry);
      rotationToParent = new YoFrameQuaternion(frameName, parentFrame, registry);
   }

   public void setOffsetToParent(FrameTuple<?, ?> translationToParent, FrameOrientation rotationToParent)
   {
      this.translationToParent.set(translationToParent);
      this.rotationToParent.set(rotationToParent);
      update();
   }

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
}
