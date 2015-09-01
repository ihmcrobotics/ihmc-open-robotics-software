package us.ihmc.robotics.referenceFrames;

import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TranslationReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -6741627181585210414L;
   public final FrameVector originVector;

   public TranslationReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, false, false, parentFrame.isZupFrame());

      originVector = new FrameVector(parentFrame);
   }

   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {      
      transformToParent.setIdentity();
      transformToParent.setTranslation(originVector.getVector());
   }

   public void updateTranslation(FrameTuple frameVector)
   {      
      originVector.set(frameVector);
      this.update();
   }
   
   public void updateTranslation(Tuple3d translation)
   {      
      originVector.set(translation);
      this.update();
   }
}  

