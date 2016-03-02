package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.geometry.transformables.Transformable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class FrameWaypoint<S extends Transformable & GeometryObject<S>, F extends FrameWaypoint<S, F>>
      extends AbstractReferenceFrameHolder implements GeometryObject<F>
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   protected final S simpleWaypoint;

   protected FrameWaypoint(S simpleWaypoint)
   {
      this.simpleWaypoint = simpleWaypoint;
   }

   public final void set(S simpleWaypoint)
   {
      this.simpleWaypoint.set(simpleWaypoint);
   }

   public final void setIncludingFrame(ReferenceFrame referenceFrame, S simpleWaypoint)
   {
      this.referenceFrame = referenceFrame;
      this.simpleWaypoint.set(simpleWaypoint);
   }

   @Override
   public void set(F other)
   {
      checkReferenceFrameMatch(other);
      simpleWaypoint.set(other.simpleWaypoint);
   }

   public void setIncludingFrame(F other)
   {
      referenceFrame = other.getReferenceFrame();
      simpleWaypoint.set(other.simpleWaypoint);
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
      simpleWaypoint.applyTransform(transform);
   }

   @Override
   public void setToZero()
   {
      simpleWaypoint.setToZero();
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      simpleWaypoint.setToZero();
   }

   @Override
   public void setToNaN()
   {
      simpleWaypoint.setToNaN();
   }

   public void setToNaN(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      simpleWaypoint.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return simpleWaypoint.containsNaN();
   }

   public final void get(S simpleWaypoint)
   {
      simpleWaypoint.set(this.simpleWaypoint);
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
      if (!simpleWaypoint.epsilonEquals(other.simpleWaypoint, epsilon))
         return false;
      return true;
   }

   S getSimpleWaypoint()
   {
      return simpleWaypoint;
   }
}
