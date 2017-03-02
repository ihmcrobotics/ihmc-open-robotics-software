package us.ihmc.graphicsDescription.instructions;

public class PyramidCubeGraphics3DInstruction extends PrimitiveGraphics3DInstruction
{
   private final double lengthX;
   private final double widthY;
   private final double heightZ;
   private final double pyramidHeight;

   public PyramidCubeGraphics3DInstruction(double lengthX, double widthY, double heightZ, double pyramidHeight)
   {
      this.lengthX = lengthX;
      this.widthY = widthY;
      this.heightZ = heightZ;
      this.pyramidHeight = pyramidHeight;
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

   public double getPyramidHeight()
   {
      return pyramidHeight;
   }
}
