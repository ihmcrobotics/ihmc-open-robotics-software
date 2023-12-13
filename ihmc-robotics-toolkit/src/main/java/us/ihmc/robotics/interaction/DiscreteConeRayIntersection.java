package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.interaction.SphereRayIntersection;

public class DiscreteConeRayIntersection
{
   private final SphereRayIntersection boundingSphereIntersection = new SphereRayIntersection();
   private final Plane3D coneBasePlaneFacingTip = new Plane3D();
   private final Plane3D coneTipPlaneFacingBase = new Plane3D();
   private final Line3D coneBaseTowardsTipAxis = new Line3D();
   private final Point3D closestConeAxisPoint = new Point3D();
   private final RigidBodyTransform coneBaseCenterTipUpTransform = new RigidBodyTransform();
   private final Point3D interpolatedPoint = new Point3D();
   private double arrowHeadRadius;
   private double arrowHeadLength;

   public void update(double coneBaseRadius,
                      double coneHeight,
                      double zOffset,
                      RigidBodyTransformReadOnly transform)
   {
      this.arrowHeadRadius = coneBaseRadius;
      this.arrowHeadLength = coneHeight;

      coneBaseCenterTipUpTransform.setToZero();
      coneBaseCenterTipUpTransform.getTranslation().addZ(zOffset);
      transform.transform(coneBaseCenterTipUpTransform);

      double boundingSphereRadius;
      if (coneBaseRadius > (0.5 * coneHeight))
         boundingSphereRadius = coneBaseRadius / Math.sin(Math.atan(2.0 * coneBaseRadius / coneHeight));
      else
         boundingSphereRadius = 0.5 * coneHeight;
      boundingSphereIntersection.update(boundingSphereRadius, zOffset + Math.signum(zOffset) * 0.5 * coneHeight, transform);
   }

   public double intersect(Line3DReadOnly pickRay, int resolution)
   {
      if (boundingSphereIntersection.intersect(pickRay))
      {
         coneBasePlaneFacingTip.setToZero();
         coneBasePlaneFacingTip.applyTransform(coneBaseCenterTipUpTransform);

         coneTipPlaneFacingBase.setToZero();
         coneTipPlaneFacingBase.getPoint().addZ(arrowHeadLength);
         coneTipPlaneFacingBase.getNormal().set(0.0, 0.0, -1.0);
         coneTipPlaneFacingBase.applyTransform(coneBaseCenterTipUpTransform);

         coneBaseTowardsTipAxis.set(coneBasePlaneFacingTip.getPoint(), coneBasePlaneFacingTip.getNormal());

         for (int i = 0; i < resolution; i++)
         {
            interpolatedPoint.interpolate(boundingSphereIntersection.getFirstIntersectionToPack(),
                                          boundingSphereIntersection.getSecondIntersectionToPack(),
                                          i / (double) resolution);

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
