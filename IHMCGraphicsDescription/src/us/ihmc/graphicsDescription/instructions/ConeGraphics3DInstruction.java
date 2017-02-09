package us.ihmc.graphicsDescription.instructions;

public class ConeGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double height;
   private final double radius;
   private final int resolution;

   public ConeGraphics3DInstruction(double height, double radius, int resolution)
   {
      this.height = height;
      this.radius = radius;
      this.resolution = resolution;
   }

   public double getHeight()
   {
      return height;
   }

   public double getRadius()
   {
      return radius;
   }

   public int getResolution()
   {
      return resolution;
   }
}
