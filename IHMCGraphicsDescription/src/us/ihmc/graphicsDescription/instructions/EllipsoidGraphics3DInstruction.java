package us.ihmc.graphicsDescription.instructions;

public class EllipsoidGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double xRadius;
   private final double yRadius;
   private final double zRadius;
   private final int resolution;

   public EllipsoidGraphics3DInstruction(double xRadius, double yRadius, double zRadius, int resolution)
   {
      this.xRadius = xRadius;
      this.yRadius = yRadius;
      this.zRadius = zRadius;
      this.resolution = resolution;
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
