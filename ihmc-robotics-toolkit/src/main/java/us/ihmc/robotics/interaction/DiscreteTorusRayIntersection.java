package us.ihmc.robotics.interaction;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;

public class DiscreteTorusRayIntersection
{
   private final Torus3D torus = new Torus3D();
   private final StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();

   public void update(double radius, double tubeRadius, RigidBodyTransformReadOnly transform)
   {
      torus.setToZero();
      torus.setRadii(radius, tubeRadius);
      torus.applyTransform(transform);

      stepCheckIsPointInsideAlgorithm.update(radius + tubeRadius, transform);
   }

   public double intersect(Line3DReadOnly pickRay, int resolution)
   {
      return stepCheckIsPointInsideAlgorithm.intersect(pickRay, resolution, torus::isPointInside);
   }

   public Point3D getClosestIntersection()
   {
      return stepCheckIsPointInsideAlgorithm.getClosestIntersection();
   }
}
