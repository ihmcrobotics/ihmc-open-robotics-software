package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

public class CapsuleRayIntersection
{
   private final SphereRayIntersection sphereIntersectionPositiveEnd = new SphereRayIntersection();
   private final SphereRayIntersection sphereIntersectionNegativeEnd = new SphereRayIntersection();
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();

   private final Vector3D tempVector = new Vector3D();
   private final Point3D tempPoint = new Point3D();

   public void setup(double radius, double length, Point3DReadOnly position, UnitVector3DReadOnly axis)
   {
      tempVector.set(axis);
      tempVector.scale(length / 2.0);
      tempPoint.set(position);
      tempPoint.add(tempVector);
      sphereIntersectionPositiveEnd.setup(radius, tempPoint);

      tempVector.set(axis);
      tempVector.scale(length / 2.0);
      tempPoint.set(position);
      tempPoint.sub(tempVector);
      sphereIntersectionNegativeEnd.setup(radius, tempPoint);

      cylinderIntersection.setup(length, radius);

//      EuclidGeometryTools.distanceFromPoint3DToLineSegment3D() // TODO: Is there a simpler way to do capsule ray intersection?
   }

   public boolean intersect(Line3DReadOnly pickRay)
   {
      boolean intersects = sphereIntersectionPositiveEnd.intersect(pickRay);

      intersects |= sphereIntersectionNegativeEnd.intersect(pickRay);

      double closestDistance = cylinderIntersection.intersect(pickRay);

      return intersects || Double.isNaN(closestDistance);
   }
}
