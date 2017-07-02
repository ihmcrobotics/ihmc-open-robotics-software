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
   public final void setToZero()
   {
      geometryObject.setToZero();
   }

   public final void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToZero();
   }

   @Override
   public final void setToNaN()
   {
      geometryObject.setToNaN();
   }

   public final void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      setToNaN();
   }

   @Override
   public final boolean containsNaN()
   {
      return geometryObject.containsNaN();
   }

   public final void set(G geometryObject)
   {
      this.geometryObject.set(geometryObject);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, G geometryObject)
   {
      this.referenceFrame = referenceFrame;
      this.geometryObject.set(geometryObject);
   }

   @Override
   public final void set(F other)
   {
      checkReferenceFrameMatch(other);
      geometryObject.set(other.getGeometryObject());
   }

   public final void setIncludingFrame(F other)
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
   public final void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      ReferenceFrame thisReferenceFrame = getReferenceFrame();
      setToZero(referenceFrame);
      changeFrame(thisReferenceFrame);
   }

   public final void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == referenceFrame)
         return;

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

   public final void changeFrameUsingTransform(ReferenceFrame desiredFrame, Transform transformToNewFrame)
   {
      geometryObject.applyTransform(transformToNewFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public final void applyTransform(Transform transform)
   {
      geometryObject.applyTransform(transform);
   }

   @Override
   public final void applyInverseTransform(Transform transform)
   {
      geometryObject.applyInverseTransform(transform);
   }

   @Override
   public final ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void get(G geometryObject)
   {
      geometryObject.set(this.geometryObject);
   }

   public final G getGeometryObject()
   {
      return geometryObject;
   }

   @Override
   public final boolean epsilonEquals(F other, double epsilon)
   {
      if (referenceFrame != other.referenceFrame)
         return false;
      return geometryObject.epsilonEquals(other.getGeometryObject(), epsilon);
   }

   @SuppressWarnings("unchecked")
   @Override
   public final boolean equals(Object obj)
   {
      try
      {
         return equals((F) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public final boolean equals(F other)
   {
      try
      {
         if (referenceFrame != other.referenceFrame)
            return false;
         return geometryObject.equals(other.getGeometryObject());
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }

   @Override
   public final String toString()
   {
      return geometryObject + "-" + referenceFrame;
   }
}
