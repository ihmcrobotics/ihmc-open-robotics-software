package us.ihmc.graphicsDescription.instructions;

public class TruncatedConeGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double height;
   private final double xBaseRadius;
   private final double yBaseRadius;
   private final double xTopRadius;
   private final double yTopRadius;
   private final int resolution;

   public TruncatedConeGraphics3DInstruction(double height, double xBaseRadius, double yBaseRadius, double xTopRadius, double yTopRadius, int resolution)
   {
      this.height = height;
      this.xBaseRadius = xBaseRadius;
      this.yBaseRadius = yBaseRadius;
      this.xTopRadius = xTopRadius;
      this.yTopRadius = yTopRadius;
      this.resolution = resolution;
   }

   public double getHeight()
   {
      return height;
   }

   public double getXBaseRadius()
   {
      return xBaseRadius;
   }

   public double getYBaseRadius()
   {
      return yBaseRadius;
   }

   public double getXTopRadius()
   {
      return xTopRadius;
   }

   public double getYTopRadius()
   {
      return yTopRadius;
   }

   public int getResolution()
   {
      return resolution;
   }
}
