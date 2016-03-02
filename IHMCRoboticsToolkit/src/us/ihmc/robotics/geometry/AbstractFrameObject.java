package us.ihmc.robotics.geometry;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractFrameObject<S extends AbstractFrameObject<S, T>, T extends GeometryObject<T>> extends AbstractReferenceFrameHolder implements FrameObject<S>
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
            transformableDataObject.applyTransform(referenceTf);
         }

         if ((desiredTf = desiredFrame.getInverseTransformToRoot()) != null)
         {
            transformableDataObject.applyTransform(desiredTf);
         }

         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   @Override
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      transformableDataObject.applyTransform(transformToNewFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transformableDataObject.applyTransform(transform);
   }
   
   @Override
   public void set(S other)
   {
      checkReferenceFrameMatch(other);
      this.transformableDataObject.set(other.transformableDataObject);
   }

   @Override
   public void setToZero()
   {
      this.transformableDataObject.setToZero();
   }

   @Override
   public void setToNaN()
   {
      this.transformableDataObject.setToNaN();      
   }

   @Override
   public boolean containsNaN()
   {
      return this.transformableDataObject.containsNaN();
   }

   @Override
   public boolean epsilonEquals(S other, double epsilon)
   {
      if (other == null) return false;
      if (referenceFrame != other.referenceFrame) return false;
      return this.transformableDataObject.epsilonEquals(other.transformableDataObject, epsilon);
   }

   @Override
   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToZero();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToNaN();
   }
   
   public void set(T other)
   {
      this.transformableDataObject.set(other);
   }
   
   public void set(ReferenceFrame referenceFrame, T other)
   {
      this.referenceFrame = referenceFrame;
      this.transformableDataObject.set(other);
   }

}
