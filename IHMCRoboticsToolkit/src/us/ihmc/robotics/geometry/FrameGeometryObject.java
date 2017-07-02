package us.ihmc.robotics.geometry;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;

public abstract class FrameGeometryObject<F extends FrameGeometryObject<F, G>, G extends GeometryObject<G>> implements ReferenceFrameHolder, GeometryObject<F>
{
   private final G geometryObject;
   protected ReferenceFrame referenceFrame;

   public FrameGeometryObject(G geometryObject)
   {
      this(ReferenceFrame.getWorldFrame(), geometryObject);
   }

   public FrameGeometryObject(ReferenceFrame referenceFrame, G geometryObject)
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

   /**
    * Sets this frame object to zero at the origin of the given reference frame, then changes back
    * to this objects current frame.
    * 
    * @param referenceFrame reference frame to set to
    */
   public void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      ReferenceFrame thisReferenceFrame = getReferenceFrame();
      setToZero(referenceFrame);
      changeFrame(thisReferenceFrame);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame != referenceFrame)
      {
         referenceFrame.verifySameRoots(desiredFrame);
         RigidBodyTransform referenceFrameTransformToRoot, desiredFrameTransformToRoot;

         if ((referenceFrameTransformToRoot = referenceFrame.getTransformToRoot()) != null)
         {
            geometryObject.applyTransform(referenceFrameTransformToRoot);
         }

         if ((desiredFrameTransformToRoot = desiredFrame.getTransformToRoot()) != null)
         {
            geometryObject.applyInverseTransform(desiredFrameTransformToRoot);
         }

         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

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
   public void applyInverseTransform(Transform transform)
   {
      geometryObject.applyInverseTransform(transform);
   }

   @Override
   public void setToZero()
   {
      geometryObject.setToZero();
   }

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
      if (other == null)
         return false;
      if (referenceFrame != other.referenceFrame)
         return false;
      return this.geometryObject.epsilonEquals(other.getGeometryObject(), epsilon);
   }

}
