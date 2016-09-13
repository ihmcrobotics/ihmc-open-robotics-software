package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CylinderShapeDescription implements CollisionShapeDescription
{
   private final double radius;
   private final double height;

   public CylinderShapeDescription(double radius, double height)
   {
      this.radius = radius;
      this.height = height;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

}
