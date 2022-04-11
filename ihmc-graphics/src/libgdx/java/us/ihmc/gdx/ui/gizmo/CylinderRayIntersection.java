package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class CylinderRayIntersection
{
   private final Cylinder3D cylinder = new Cylinder3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private boolean intersects = false;

   public void setup(double length, double radius)
   {
      cylinder.setToZero();
      cylinder.setSize(length, radius);
   }

   public void setup(double length, double radius, RigidBodyTransformReadOnly transform)
   {
      cylinder.setToZero();
      cylinder.setSize(length, radius);
      cylinder.applyTransform(transform);
   }

   public void setup(double length, double radius, double zOffset, RigidBodyTransformReadOnly transform)
   {
      cylinder.setToZero();
      cylinder.setSize(length, radius);
      cylinder.getPosition().addZ(zOffset);
      cylinder.applyTransform(transform);
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
      intersects = numberOfIntersections == 2;
      return intersects ? firstIntersectionToPack.distance(pickRay.getPoint()) : Double.NaN;
   }

   public Point3D getClosestIntersection()
   {
      return firstIntersectionToPack;
   }

   public Cylinder3D getCylinder()
   {
      return cylinder;
   }

   public boolean getIntersects()
   {
      return intersects;
   }
}
