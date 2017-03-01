package us.ihmc.robotics.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractFrameObject<F extends AbstractFrameObject<F, G>, G extends GeometryObject<G>> extends AbstractReferenceFrameHolder implements FrameObject<F>
{
   private final G geometryObject;
   protected ReferenceFrame referenceFrame;

   public AbstractFrameObject(G geometryObject)
   {
      this(ReferenceFrame.getWorldFrame(), geometryObject);
   }

   public AbstractFrameObject(ReferenceFrame referenceFrame, G geometryObject)
   {
      this.geometryObject = geometryObject;
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void set(F other)
   {
      checkReferenceFrameMatch(other);
      geometryObject.set(other.getGeometryObject());
   }

   public void setIncludingFrame(F other)
   {
      referenceFrame = other.getReferenceFrame();
      geometryObject.set(other.getGeometryObject());
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
   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, Transform transformToNewFrame)
   {
      geometryObject.applyTransform(transformToNewFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      geometryObject.applyTransform(transform);
   }

   @Override
   public void setToZero()
   {
      geometryObject.setToZero();
   }

   @Override
   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToZero();
   }

   @Override
   public void setToNaN()
   {
      geometryObject.setToNaN();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return geometryObject.containsNaN();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void set(G geometryObject)
   {
      this.geometryObject.set(geometryObject);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, G geometryObject)
   {
      this.referenceFrame = referenceFrame;
      this.geometryObject.set(geometryObject);
   }

   public void get(G geometryObject)
   {
      geometryObject.set(this.geometryObject);
   }

   public G getGeometryObject()
   {
      return geometryObject;
   }

   @Override
   public boolean epsilonEquals(F other, double epsilon)
   {
      if (other == null) return false;
      if (referenceFrame != other.referenceFrame) return false;
      return this.geometryObject.epsilonEquals(other.getGeometryObject(), epsilon);
   }

}
