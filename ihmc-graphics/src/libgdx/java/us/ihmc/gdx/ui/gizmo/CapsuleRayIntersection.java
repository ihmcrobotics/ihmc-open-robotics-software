package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

public class CapsuleRayIntersection
{
   private final SphereRayIntersection sphereIntersectionPositiveEnd = new SphereRayIntersection();
   private final SphereRayIntersection sphereIntersectionNegativeEnd = new SphereRayIntersection();
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();

   private final Vector3D tempVector = new Vector3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void setup(double radius, double length, Point3DReadOnly position, UnitVector3DReadOnly axis, RigidBodyTransform transformToWorld)
   {
      tempTransform.set(transformToWorld);
      tempVector.set(axis);
      transformToWorld.transform(tempVector);
      tempVector.scale(length / 2.0);
      tempTransform.appendTranslation(tempVector);
      sphereIntersectionPositiveEnd.setup(radius, tempTransform);

      tempVector.scale(-2.0);
      tempTransform.appendTranslation(tempVector);
      sphereIntersectionNegativeEnd.setup(radius, tempTransform);

      cylinderIntersection.setup(length, radius, transformToWorld);

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
