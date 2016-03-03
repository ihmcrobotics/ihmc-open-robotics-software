package us.ihmc.robotics.geometry;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractFrameObject<S extends AbstractFrameObject<S, G>, G extends GeometryObject<G>> extends AbstractReferenceFrameHolder implements FrameObject<S>
{
   private final G geometryObject;
   protected ReferenceFrame referenceFrame;

   public AbstractFrameObject(ReferenceFrame referenceFrame, G transformableDataObject)
   {
      this.geometryObject = transformableDataObject;
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
         RigidBodyTransform referenceFrameTransformToRoot, rootTransformToDesiredFrame;

         if ((referenceFrameTransformToRoot = referenceFrame.getTransformToRoot()) != null)
         {
            geometryObject.applyTransform(referenceFrameTransformToRoot);
         }

         if ((rootTransformToDesiredFrame = desiredFrame.getInverseTransformToRoot()) != null)
         {
            geometryObject.applyTransform(rootTransformToDesiredFrame);
         }

         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   @Override
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
   {
      geometryObject.applyTransform(transformToNewFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      geometryObject.applyTransform(transform);
   }
   
   @Override
   public void set(S other)
   {
      checkReferenceFrameMatch(other);
      this.geometryObject.set(other.getGeometryObject());
   }

   @Override
   public void setToZero()
   {
      this.geometryObject.setToZero();
   }

   @Override
   public void setToNaN()
   {
      this.geometryObject.setToNaN();      
   }

   @Override
   public boolean containsNaN()
   {
      return this.geometryObject.containsNaN();
   }

   @Override
   public boolean epsilonEquals(S other, double epsilon)
   {
      if (other == null) return false;
      if (referenceFrame != other.referenceFrame) return false;
      return this.geometryObject.epsilonEquals(other.getGeometryObject(), epsilon);
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

   public void set(G other)
   {
      this.geometryObject.set(other);
   }

   public void set(ReferenceFrame referenceFrame, G other)
   {
      this.referenceFrame = referenceFrame;
      this.geometryObject.set(other);
   }
   
   public G getGeometryObject()
   {
      return geometryObject;
   }

}
