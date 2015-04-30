package us.ihmc.ihmcPerception.linemod;

import java.awt.image.BufferedImage;

public class OrganizedPointCloud
{
   public int width;
   public int height;
   public float[] xyzrgb;
   
   public OrganizedPointCloud(int width, int height, float[] xyzrgb)
   {
      this.width=width;
      this.height=height;
      this.xyzrgb=xyzrgb;
   }
   
   public BufferedImage getRGBImage()
   {
      BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_4BYTE_ABGR);
      for(int h=0;h<height;h++)
         for(int w=0;w<width;w++)
            image.setRGB(w, h, Float.floatToRawIntBits(xyzrgb[4*(h*width+w)+3]));
      return image;
   }
   

}
