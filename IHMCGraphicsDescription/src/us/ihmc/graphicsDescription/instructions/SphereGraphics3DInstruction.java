package us.ihmc.graphicsDescription.instructions;

public class SphereGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double radius;
   private final int resolution;

   public SphereGraphics3DInstruction(double radius, int resolution)
   {
      super();
      this.radius = radius;
      this.resolution = resolution;
   }

   public int getResolution()
   {
      return resolution;
   }

   public double getRadius()
   {
      return radius;
   }

}
