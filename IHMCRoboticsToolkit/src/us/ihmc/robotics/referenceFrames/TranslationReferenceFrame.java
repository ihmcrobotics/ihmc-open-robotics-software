package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;

public class TranslationReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -6741627181585210414L;
   public final FrameVector originVector;

   public TranslationReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, false, false, parentFrame.isZupFrame());

      originVector = new FrameVector(parentFrame);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {      
      transformToParent.setIdentity();
      transformToParent.setTranslation(originVector.getVector());
   }

   public void updateTranslation(FrameTuple<?, ?> frameVector)
   {      
      originVector.set(frameVector);
      this.update();
   }
   
   public void updateTranslation(Tuple3DBasics translation)
   {      
      originVector.set(translation);
      this.update();
   }
}  
