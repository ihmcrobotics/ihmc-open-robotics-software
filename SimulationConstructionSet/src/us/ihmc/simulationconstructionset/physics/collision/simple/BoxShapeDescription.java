package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class BoxShapeDescription implements CollisionShapeDescription
{
   private final double halfLengthX;
   private final double halfWidthY;
   private final double halfHeightZ;

   public BoxShapeDescription(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      this.halfLengthX = halfLengthX;
      this.halfWidthY = halfWidthY;
      this.halfHeightZ = halfHeightZ;
   }

   public double getHalfLengthX()
   {
      return halfLengthX;
   }

   public double getHalfWidthY()
   {
      return halfWidthY;
   }

   public double getHalfHeightZ()
   {
      return halfHeightZ;
   }

}
