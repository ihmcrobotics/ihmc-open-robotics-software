package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class DiscreteArrowRayIntersection
{
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
   private final DiscreteConeRayIntersection coneRayIntersection = new DiscreteConeRayIntersection();
   private final Point3D intersection = new Point3D();

   public void update(double arrowBodyLength,
                      double arrowBodyRadius,
                      double arrowHeadRadius,
                      double arrowHeadLength,
                      double zOffset,
                      RigidBodyTransformReadOnly transform)
   {
      cylinderIntersection.update(arrowBodyLength, arrowBodyRadius, zOffset, transform);
      coneRayIntersection.update(arrowHeadRadius, arrowHeadLength, zOffset + Math.signum(zOffset) * 0.5 * arrowBodyLength, transform);
   }

   public double intersect(Line3DReadOnly pickRay, int resolution, boolean collideArrowHead)
   {
      double cylinderDistance = cylinderIntersection.intersect(pickRay);

      if (collideArrowHead)
      {
         double coneDistance = coneRayIntersection.intersect(pickRay, resolution);

         if (Double.isNaN(cylinderDistance) && Double.isNaN(coneDistance))
         {
            return Double.NaN;
         }
         else if (Double.isNaN(cylinderDistance) || coneDistance < cylinderDistance)
         {
            intersection.set(coneRayIntersection.getIntersectionPoint());
            return coneDistance;
         }
         else
         {
            intersection.set(cylinderIntersection.getClosestIntersection());
            return cylinderDistance;
         }
      }
      else
      {
         if (!Double.isNaN(cylinderDistance))
         {
            intersection.set(cylinderIntersection.getClosestIntersection());
         }
         return cylinderDistance;
      }
   }

   public Point3D getIntersection()
   {
      return intersection;
   }
}
