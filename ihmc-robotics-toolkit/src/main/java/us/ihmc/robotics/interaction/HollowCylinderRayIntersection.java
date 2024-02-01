package us.ihmc.robotics.interaction;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class HollowCylinderRayIntersection
{
   private final Cylinder3D cylinder = new Cylinder3D();
   private final Cylinder3D hollowPart = new Cylinder3D();
   private final Line3D cylinderAxis = new Line3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private final Point3D secondHollowIntersectionToPack = new Point3D();
   private final Point3D closestCollisionPoint = new Point3D();
   private double innerRadius;

   public void update(double length, double outerRadius, double innerRadius, double zOffset, RigidBodyTransformReadOnly transform)
   {
      this.innerRadius = innerRadius;
      cylinder.setToZero();
      cylinder.setSize(length, outerRadius);
      cylinder.getPosition().addZ(zOffset);
      cylinder.applyTransform(transform);

      cylinderAxis.getPoint().setToZero();
      cylinderAxis.getDirection().set(Axis3D.Z);
      cylinderAxis.applyTransform(transform);

      hollowPart.setToZero();
      hollowPart.setSize(length, innerRadius);
      hollowPart.getPosition().addZ(zOffset);
      hollowPart.applyTransform(transform);
   }

   public double intersect(Line3DReadOnly pickRay)
   {
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinder.getLength(),
                                                                                            cylinder.getRadius(),
                                                                                            cylinder.getPosition(),
                                                                                            cylinder.getAxis(),
                                                                                            pickRay.getPoint(),
                                                                                            pickRay.getDirection(),
                                                                                            firstIntersectionToPack,
                                                                                            secondIntersectionToPack);
      boolean intersectsBigCylinder = numberOfIntersections == 2;

      closestCollisionPoint.setToNaN();
      if (intersectsBigCylinder)
      {
         double firstDistanceToAxis = cylinderAxis.distance(firstIntersectionToPack);
         double secondDistanceToAxis = cylinderAxis.distance(secondIntersectionToPack);
         boolean firstIntersectionIsInsideHollowPart = firstDistanceToAxis < innerRadius;
         boolean secondIntersectionIsInsideHollowPart = secondDistanceToAxis < innerRadius;

         if (!firstIntersectionIsInsideHollowPart) // the first intersection is the closest intersection
         {
            closestCollisionPoint.set(firstIntersectionToPack);
         }
         else if (!secondIntersectionIsInsideHollowPart) // comes in through the hole and hits the inner wall
         {
            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(hollowPart.getLength(),
                                                                      hollowPart.getRadius(),
                                                                      hollowPart.getPosition(),
                                                                      hollowPart.getAxis(),
                                                                      pickRay.getPoint(),
                                                                      pickRay.getDirection(),
                                                                      null,
                                                                      secondHollowIntersectionToPack);
            closestCollisionPoint.set(secondHollowIntersectionToPack);
         }
         // else the pick ray passes through the hole without collision
      }

      return !closestCollisionPoint.containsNaN() ? firstIntersectionToPack.distance(pickRay.getPoint()) : Double.NaN;
   }

   public Point3D getClosestIntersection()
   {
      return closestCollisionPoint;
   }

   public Cylinder3D getCylinder()
   {
      return cylinder;
   }
}
