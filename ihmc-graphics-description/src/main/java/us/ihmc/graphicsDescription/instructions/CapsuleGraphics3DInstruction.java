package us.ihmc.graphicsDescription.instructions;

public class CapsuleGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double height;
   private final double xRadius;
   private final double yRadius;
   private final double zRadius;
   private final int resolution;

   public CapsuleGraphics3DInstruction(double height, double xRadius, double yRadius, double zRadius, int resolution)
   {
      this.height = height;
      this.xRadius = xRadius;
      this.yRadius = yRadius;
      this.zRadius = zRadius;
      this.resolution = resolution;
   }

   public double getHeight()
   {
      return height;
   }

   public double getXRadius()
   {
      return xRadius;
   }

   public double getYRadius()
   {
      return yRadius;
   }

   public double getZRadius()
   {
      return zRadius;
   }

   public int getResolution()
   {
      return resolution;
   }
}
