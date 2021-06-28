package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class DiscreteTorusRayIntersection
{
   private final Torus3D torus = new Torus3D();
   private final Sphere3D boundingSphere = new Sphere3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private final Point3D rayOriginInTorusFrame = new Point3D();
   private final Point3D interpolatedPoint = new Point3D();

   public void setupTorus(double radius, double tubeRadius, RigidBodyTransformReadOnly transform)
   {
      torus.setToZero();
      torus.setRadii(radius, tubeRadius);
      torus.applyTransform(transform);

      boundingSphere.setToZero();
      boundingSphere.setRadius(radius + tubeRadius);
      boundingSphere.applyTransform(transform);
   }

   public double intersect(Line3DReadOnly pickRay, int resolution)
   {
      rayOriginInTorusFrame.setX(pickRay.getPoint().getX() - boundingSphere.getPosition().getX());
      rayOriginInTorusFrame.setY(pickRay.getPoint().getY() - boundingSphere.getPosition().getY());
      rayOriginInTorusFrame.setZ(pickRay.getPoint().getZ() - boundingSphere.getPosition().getZ());
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(),
                                                                                             rayOriginInTorusFrame,
                                                                                             pickRay.getDirection(),
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);
      if (numberOfIntersections == 2)
      {
         firstIntersectionToPack.add(boundingSphere.getPosition());
         secondIntersectionToPack.add(boundingSphere.getPosition());
         for (int i = 0; i < resolution; i++)
         {
            interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / (double) resolution);
            if (torus.isPointInside(interpolatedPoint))
            {
               return interpolatedPoint.distance(pickRay.getPoint());
            }
         }
      }

      return Double.NaN;
   }

   public Point3D getClosestIntersection()
   {
      return interpolatedPoint;
   }
}
