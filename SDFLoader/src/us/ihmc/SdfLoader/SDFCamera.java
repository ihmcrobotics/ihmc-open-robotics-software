package us.ihmc.SdfLoader;

public class SDFCamera
{
   private final int width, height;

   public SDFCamera(int width, int height)
   {
      this.width = width;
      this.height = height;
   }

   public int getWidth()
   {
      return width;
   }

   public int getHeight()
   {
      return height;
      
   }
}
