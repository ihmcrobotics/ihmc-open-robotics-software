package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

public class CapsuleRayIntersection
{
   private final SphereRayIntersection sphereIntersectionPositiveEnd = new SphereRayIntersection();
   private final SphereRayIntersection sphereIntersectionNegativeEnd = new SphereRayIntersection();
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();

   private final FramePoint3D tempSphereCenter = new FramePoint3D();

   public void update(double radius, double length, Point3DReadOnly position, UnitVector3DReadOnly axis, ReferenceFrame frameAfterJoint)
   {
      cylinderIntersection.update(length, radius, position, axis, frameAfterJoint);

      tempSphereCenter.setIncludingFrame(frameAfterJoint, cylinderIntersection.getCylinder().getTopCenter());
      tempSphereCenter.changeFrame(ReferenceFrame.getWorldFrame());
      sphereIntersectionPositiveEnd.update(radius, tempSphereCenter);

      tempSphereCenter.setIncludingFrame(frameAfterJoint, cylinderIntersection.getCylinder().getBottomCenter());
      tempSphereCenter.changeFrame(ReferenceFrame.getWorldFrame());
      sphereIntersectionNegativeEnd.update(radius, tempSphereCenter);

//      EuclidGeometryTools.distanceFromPoint3DToLineSegment3D() // TODO: Is there a simpler way to do capsule ray intersection?
   }

   public boolean intersect(Line3DReadOnly pickRay)
   {
      boolean intersects = false;
      intersects |= sphereIntersectionPositiveEnd.intersect(pickRay);
      intersects |= sphereIntersectionNegativeEnd.intersect(pickRay);
      double closestDistance = cylinderIntersection.intersect(pickRay);
      intersects |= !Double.isNaN(closestDistance);
      return intersects;
   }

   public double getDistanceToCollision(Line3DReadOnly pickRay)
   {
      double distanceToCollision = Double.POSITIVE_INFINITY;
      if (sphereIntersectionPositiveEnd.getIntersects())
      {
         double distance = pickRay.getPoint().distance(sphereIntersectionPositiveEnd.getFirstIntersectionToPack());
         if (distance < distanceToCollision)
         {
            distanceToCollision = distance;
         }
      }
      if (sphereIntersectionNegativeEnd.getIntersects())
      {
         double distance = pickRay.getPoint().distance(sphereIntersectionNegativeEnd.getFirstIntersectionToPack());
         if (distance < distanceToCollision)
         {
            distanceToCollision = distance;
         }
      }
      if (cylinderIntersection.getIntersects())
      {
         double distance = pickRay.getPoint().distance(cylinderIntersection.getClosestIntersection());
         if (distance < distanceToCollision)
         {
            distanceToCollision = distance;
         }
      }
      return distanceToCollision;
   }
}
