package us.ihmc.graphicsDescription.instructions;

public class CylinderGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double radius;
   private final double height;
   private final int resolution;

   public CylinderGraphics3DInstruction(double radius, double height, int resolution)
   {
      this.radius = radius;
      this.height = height;
      this.resolution = resolution;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

   public int getResolution()
   {
      return resolution;
   }
}
