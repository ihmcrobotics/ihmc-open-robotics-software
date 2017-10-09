package us.ihmc.graphicsDescription.instructions;

public class ArcTorusGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double startAngle;
   private final double endAngle;
   private final double majorRadius;
   private final double minorRadius;
   private final int resolution;

   public ArcTorusGraphics3DInstruction(double startAngle, double endAngle, double majorRadius, double minorRadius, int resolution)
   {
      this.startAngle = startAngle;
      this.endAngle = endAngle;
      this.majorRadius = majorRadius;
      this.minorRadius = minorRadius;
      this.resolution = resolution;
   }

   public double getStartAngle()
   {
      return startAngle;
   }

   public double getEndAngle()
   {
      return endAngle;
   }

   public double getMajorRadius()
   {
      return majorRadius;
   }

   public double getMinorRadius()
   {
      return minorRadius;
   }

   public int getResolution()
   {
      return resolution;
   }
}
