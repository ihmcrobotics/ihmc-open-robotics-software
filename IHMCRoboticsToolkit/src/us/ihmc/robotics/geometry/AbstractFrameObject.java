package us.ihmc.robotics.geometry;

import us.ihmc.robotics.geometry.transformables.TransformableDataObject;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractFrameObject<T extends TransformableDataObject> extends AbstractReferenceFrameHolder implements FrameObject
{
   protected final T transformableDataObject;
   protected ReferenceFrame referenceFrame;

   public AbstractFrameObject(ReferenceFrame referenceFrame, T transformableDataObject)
   {
      this.transformableDataObject = transformableDataObject;
      this.referenceFrame = referenceFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame != referenceFrame)
      {
         referenceFrame.verifySameRoots(desiredFrame);
         RigidBodyTransform referenceTf, desiredTf;

         if ((referenceTf = referenceFrame.getTransformToRoot()) != null)
         {
            transformableDataObject.transform(referenceTf);
         }

         if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
         {
            transformableDataObject.transform(desiredTf);
         }

         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   @Override
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      transformableDataObject.transform(transformToNewFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transformableDataObject.transform(transform);
   }
}
