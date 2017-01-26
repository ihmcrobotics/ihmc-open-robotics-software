package us.ihmc.robotics.robotDescription;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class SphereDescriptionReadOnly implements ConvexShapeDescription
{
   private final double radius;
   private final RigidBodyTransform rigidBodyTransform;

   public SphereDescriptionReadOnly(double radius, RigidBodyTransform rigidBodyTransform)
   {
      this.radius = radius;
      this.rigidBodyTransform = new RigidBodyTransform(rigidBodyTransform);
   }

   public double getRadius()
   {
      return radius;
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(rigidBodyTransform);
   }

}
