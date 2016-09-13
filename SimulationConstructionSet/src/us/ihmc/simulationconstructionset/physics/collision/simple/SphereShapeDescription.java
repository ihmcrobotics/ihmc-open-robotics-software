package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class SphereShapeDescription implements CollisionShapeDescription
{
   private final double radius;

   public SphereShapeDescription(double radius)
   {
      this.radius = radius;
   }

   public double getRadius()
   {
      return radius;
   }

}
