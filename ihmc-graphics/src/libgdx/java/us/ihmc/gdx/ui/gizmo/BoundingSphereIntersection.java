package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class BoundingSphereIntersection
{
   private final Sphere3D boundingSphere = new Sphere3D();
   private final Point3D rayOriginInSphereFrame = new Point3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();

   public void setup(double radius, RigidBodyTransformReadOnly transform)
   {
      setup(radius, 0.0, transform);
   }

   public void setup(double radius, double zOffset, RigidBodyTransformReadOnly transform)
   {
      boundingSphere.setToZero();
      boundingSphere.setRadius(radius);
      boundingSphere.getPosition().addZ(zOffset);
      boundingSphere.applyTransform(transform);
   }

   public void setup(double radius, Point3DReadOnly offset, RigidBodyTransformReadOnly transform)
   {
      boundingSphere.setToZero();
      boundingSphere.setRadius(radius);
      boundingSphere.getPosition().add(offset);
      boundingSphere.applyTransform(transform);
   }

   public void setup(double radius, Point3DReadOnly positionInWorld)
   {
      boundingSphere.setToZero();
      boundingSphere.setRadius(radius);
      boundingSphere.getPosition().set(positionInWorld);
   }

   public boolean intersect(Line3DReadOnly pickRay)
   {
      rayOriginInSphereFrame.setX(pickRay.getPoint().getX() - boundingSphere.getPosition().getX());
      rayOriginInSphereFrame.setY(pickRay.getPoint().getY() - boundingSphere.getPosition().getY());
      rayOriginInSphereFrame.setZ(pickRay.getPoint().getZ() - boundingSphere.getPosition().getZ());
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(), rayOriginInSphereFrame,
                                                                                             pickRay.getDirection(),
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);
      firstIntersectionToPack.add(boundingSphere.getPosition());
      secondIntersectionToPack.add(boundingSphere.getPosition());
      return numberOfIntersections == 2;
   }

   public Point3DReadOnly getFirstIntersectionToPack()
   {
      return firstIntersectionToPack;
   }

   public Point3DReadOnly getSecondIntersectionToPack()
   {
      return secondIntersectionToPack;
   }
}
