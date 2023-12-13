package us.ihmc.robotics.interaction;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * This class is a holder for all the variables needed to use
 * {@link FrameShape3DReadOnly#evaluatePoint3DCollision(FramePoint3DReadOnly, Point3DBasics, Vector3DBasics)}.
 */
public class PointCollidable
{
   private final FrameShape3DReadOnly shape;
   /** We need to make the frame match, but don't want to modify the caller's. */
   private final FramePoint3D pickPoint = new FramePoint3D();
   private final FramePoint3D closestPointOnSurface = new FramePoint3D();
   private final FrameVector3D normalAtClosestPointOnSurface = new FrameVector3D();
   /** Negative if inside. */
   private double signedDistanceToSurface;

   public PointCollidable(FrameShape3DReadOnly shape)
   {
      this.shape = shape;
   }

   public boolean collide(FramePoint3DReadOnly callerPickPoint)
   {
      pickPoint.setIncludingFrame(callerPickPoint);
      pickPoint.changeFrame(shape.getReferenceFrame());

      closestPointOnSurface.changeFrame(pickPoint.getReferenceFrame());
      normalAtClosestPointOnSurface.changeFrame(pickPoint.getReferenceFrame());

      boolean isInsideOrOnSurface = shape.evaluatePoint3DCollision(pickPoint, closestPointOnSurface, normalAtClosestPointOnSurface);
      double absoluteDistance = closestPointOnSurface.distance(pickPoint);
      signedDistanceToSurface = isInsideOrOnSurface ? -absoluteDistance : absoluteDistance;

      closestPointOnSurface.changeFrame(ReferenceFrame.getWorldFrame());
      normalAtClosestPointOnSurface.changeFrame(ReferenceFrame.getWorldFrame());

      return isInsideOrOnSurface;
   }

   /**
    * In world frame.
    */
   public FramePoint3DReadOnly getClosestPointOnSurface()
   {
      return closestPointOnSurface;
   }

   public double getSignedDistanceToSurface()
   {
      return signedDistanceToSurface;
   }
}
