package us.ihmc.graphicsDescription.instructions;

public class CubeGraphics3DInstruction extends Graphics3DInstruction
{
   private final boolean centeredInTheCenter; // If false, then center in the very middle of the cube.
   private final double length;
   private final double width;
   private final double height;
   private boolean[] textureFaces;

   public CubeGraphics3DInstruction(double length, double width, double height, boolean centeredInTheCenter)
   {
      super();
      
      this.length = length;
      this.width = width;
      this.height = height;
      this.centeredInTheCenter = centeredInTheCenter;
   }

   public boolean getCenteredInTheCenter()
   {
      return centeredInTheCenter;
   }

   public double getLength()
   {
      return length;
   }

   public double getWidth()
   {
      return width;
   }

   public double getHeight()
   {
      return height;
   }

   public void setTextureFaces(boolean[] textureFaces)
   {
      this.textureFaces = textureFaces;
   }

   public boolean[] getTextureFaces()
   {
      return textureFaces;
   }

   //   public String toString()
   //   {
   //      String ret = "\t\t\t<Cube>\n\t\t\t\t<Name>" + fileName + "</Name>\n";
   //      if (getAppearance() != null)
   //         ret += getAppearance().toString();
   //      ret += "\t\t\t</Add3DSFile>\n";
   //      return ret;
   //   }

}
