package us.ihmc.graphicsDescription.instructions;

public class WedgeGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double lengthX;
   private final double widthY;
   private final double heightZ;

   public WedgeGraphics3DInstruction(double lengthX, double widthY, double heightZ)
   {
      this.lengthX = lengthX;
      this.widthY = widthY;
      this.heightZ = heightZ;
   }

   public double getLengthX()
   {
      return lengthX;
   }

   public double getWidthY()
   {
      return widthY;
   }

   public double getHeightZ()
   {
      return heightZ;
   }
}
