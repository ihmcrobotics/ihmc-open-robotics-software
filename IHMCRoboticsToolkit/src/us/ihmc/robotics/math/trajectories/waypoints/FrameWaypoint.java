package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class FrameWaypoint<F extends FrameWaypoint<F, G>, G extends GeometryObject<G>> extends AbstractReferenceFrameHolder implements GeometryObject<F>
{
   private final G geometryObject;
   protected ReferenceFrame referenceFrame;

   public FrameWaypoint(G simpleWaypoint)
   {
      this(simpleWaypoint, ReferenceFrame.getWorldFrame());
   }
   
   public FrameWaypoint(G simpleWaypoint, ReferenceFrame referenceFrame)
   {
      this.geometryObject = simpleWaypoint;
      this.referenceFrame = referenceFrame;
   }

   public final void set(G simpleWaypoint)
   {
      this.geometryObject.set(simpleWaypoint);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, G simpleWaypoint)
   {
      this.referenceFrame = referenceFrame;
      this.geometryObject.set(simpleWaypoint);
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

   public final void changeFrame(ReferenceFrame referenceFrame)
   {
      // this is in the correct frame already
      if (referenceFrame == this.referenceFrame)
      {
         return;
      }

      this.referenceFrame.verifySameRoots(referenceFrame);
      RigidBodyTransform referenceTf, desiredTf;

      if ((referenceTf = this.referenceFrame.getTransformToRoot()) != null)
      {
         applyTransform(referenceTf);
      }

      if ((desiredTf = referenceFrame.getInverseTransformToRoot()) != null)
      {
         applyTransform(desiredTf);
      }

      this.referenceFrame = referenceFrame;
   }

   @Override
   public final void applyTransform(RigidBodyTransform transform)
   {
      geometryObject.applyTransform(transform);
   }

   @Override
   public void setToZero()
   {
      geometryObject.setToZero();
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      geometryObject.setToZero();
   }

   @Override
   public void setToNaN()
   {
      geometryObject.setToNaN();
   }

   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      geometryObject.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return geometryObject.containsNaN();
   }

   public final void get(G simpleWaypoint)
   {
      simpleWaypoint.set(this.geometryObject);
   }

   @Override
   public final ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public boolean epsilonEquals(F other, double epsilon)
   {
      if (referenceFrame != other.getReferenceFrame())
         return false;
      if (!geometryObject.epsilonEquals(other.getGeometryObject(), epsilon))
         return false;
      return true;
   }

   G getGeometryObject()
   {
      return geometryObject;
   }
}
