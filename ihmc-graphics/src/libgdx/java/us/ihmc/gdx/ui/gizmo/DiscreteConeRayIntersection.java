package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class DiscreteConeRayIntersection
{
   private final Sphere3D boundingSphere = new Sphere3D();
   private final Plane3D coneBasePlaneFacingTip = new Plane3D();
   private final Plane3D coneTipPlaneFacingBase = new Plane3D();
   private final Line3D coneBaseTowardsTipAxis = new Line3D();
   private final Point3D closestConeAxisPoint = new Point3D();
   private final RigidBodyTransform coneBaseCenterTipUpTransform = new RigidBodyTransform();
   private final Point3D rayOriginInSphereFrame = new Point3D();
   private final Point3D interpolatedPoint = new Point3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();
   private double arrowHeadRadius;
   private double arrowHeadLength;

   public void setupCone(double coneBaseRadius,
                         double coneHeight,
                         double zOffset,
                         RigidBodyTransformReadOnly transform)
   {
      this.arrowHeadRadius = coneBaseRadius;
      this.arrowHeadLength = coneHeight;

      coneBaseCenterTipUpTransform.setToZero();
      coneBaseCenterTipUpTransform.getTranslation().addZ(zOffset);
      transform.transform(coneBaseCenterTipUpTransform);

      boundingSphere.setToZero();
      double boundingSphereRadius;
      if (coneBaseRadius > (0.5 * coneHeight))
         boundingSphereRadius = coneBaseRadius / Math.sin(Math.atan(2.0 * coneBaseRadius / coneHeight));
      else
         boundingSphereRadius = 0.5 * coneHeight;
      boundingSphere.setRadius(boundingSphereRadius);
      boundingSphere.getPosition().addZ(zOffset + Math.signum(zOffset) * 0.5 * coneHeight);
      boundingSphere.applyTransform(transform);
   }

   public double intersect(Line3DReadOnly pickRay, int resolution)
   {
      rayOriginInSphereFrame.setX(pickRay.getPoint().getX() - boundingSphere.getPosition().getX());
      rayOriginInSphereFrame.setY(pickRay.getPoint().getY() - boundingSphere.getPosition().getY());
      rayOriginInSphereFrame.setZ(pickRay.getPoint().getZ() - boundingSphere.getPosition().getZ());
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(),
                                                                                             boundingSphere.getRadius(),
                                                                                             rayOriginInSphereFrame,
                                                                                             pickRay.getDirection(),
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);
      if (numberOfIntersections == 2)
      {
         firstIntersectionToPack.add(boundingSphere.getPosition());
         secondIntersectionToPack.add(boundingSphere.getPosition());

         coneBasePlaneFacingTip.setToZero();
         coneBasePlaneFacingTip.applyTransform(coneBaseCenterTipUpTransform);

         coneTipPlaneFacingBase.setToZero();
         coneTipPlaneFacingBase.getPoint().addZ(arrowHeadLength);
         coneTipPlaneFacingBase.getNormal().set(0.0, 0.0, -1.0);
         coneTipPlaneFacingBase.applyTransform(coneBaseCenterTipUpTransform);

         coneBaseTowardsTipAxis.set(coneBasePlaneFacingTip.getPoint(), coneBasePlaneFacingTip.getNormal());

         for (int i = 0; i < resolution; i++)
         {
            interpolatedPoint.interpolate(firstIntersectionToPack, secondIntersectionToPack, i / (double) resolution);

            if (coneBasePlaneFacingTip.isOnOrAbove(interpolatedPoint) && coneTipPlaneFacingBase.isOnOrAbove(interpolatedPoint))
            {
               coneBaseTowardsTipAxis.orthogonalProjection(interpolatedPoint, closestConeAxisPoint);
               double distanceFromBase = closestConeAxisPoint.distance(coneBasePlaneFacingTip.getPoint());
               double radiusBoundsAtTier = EuclidCoreTools.interpolate(arrowHeadRadius, 0.0, distanceFromBase / arrowHeadLength);
               if (closestConeAxisPoint.distance(interpolatedPoint) <= radiusBoundsAtTier)
               {
                  return interpolatedPoint.distance(pickRay.getPoint());
               }
            }
         }
      }

      return Double.NaN;
   }

   public Point3D getIntersectionPoint()
   {
      return interpolatedPoint;
   }
}
