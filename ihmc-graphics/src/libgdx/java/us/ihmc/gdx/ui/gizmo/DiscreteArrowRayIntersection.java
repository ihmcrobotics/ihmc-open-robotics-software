package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;

public class DiscreteArrowRayIntersection
{
   private final Cylinder3D arrowBaseCollisionCylinder = new Cylinder3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();

   public void setupShapes(double arrowBodyLength,
                           double arrowBodyRadius,
                           double arrowSpacing,
                           RobotSide arrowDirection,
                           RigidBodyTransformReadOnly transform)
   {
      arrowBaseCollisionCylinder.setToZero();
      arrowBaseCollisionCylinder.setSize(arrowBodyLength, arrowBodyRadius);
      arrowBaseCollisionCylinder.getPosition().addZ(arrowDirection.negateIfRightSide(0.5 * arrowSpacing + 0.5 * arrowBodyLength));
      arrowBaseCollisionCylinder.applyTransform(transform);
   }

   public double intersect(Line3DReadOnly pickRay, int resolution)
   {
      // collide arrow body cylinder
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(arrowBaseCollisionCylinder.getLength(),
                                                                                            arrowBaseCollisionCylinder.getRadius(),
                                                                                            arrowBaseCollisionCylinder.getPosition(),
                                                                                            arrowBaseCollisionCylinder.getAxis(),
                                                                                            pickRay.getPoint(),
                                                                                            pickRay.getDirection(),
                                                                                            firstIntersectionToPack,
                                                                                            secondIntersectionToPack);
      boolean collidedWithArrowBase = numberOfIntersections == 2;



      return Double.NaN;
   }
}
