package us.ihmc.ihmcPerception.linemod;

import java.awt.image.BufferedImage;
import java.io.Serializable;

import us.ihmc.euclid.tuple3D.Point3D;

public class OrganizedPointCloud implements Serializable
{
   /**
    * 
    */
   private static final long serialVersionUID = 5582206379721483971L;
   public int width;
   public int height;
   public float[] xyzrgb;
   
   public OrganizedPointCloud(int width, int height, float[] xyzrgb)
   {
      this.width=width;
      this.height=height;
      this.xyzrgb=xyzrgb;
   }
   
   public float[][] getDepthImage()
   {
      final int zOffset=2;
      float[][] depthImage = new float[height][width];
      for(int i=0;i<height;i++)
         for(int j=0;j<width;j++)
            depthImage[i][j] = xyzrgb[(i*width+j)*4+zOffset];
      return depthImage;
            
   }
   
   public Point3D getPoint(int w, int h)
   {
      int base=4*(h*width+w);
      return new Point3D(xyzrgb[base+0], xyzrgb[base+1], xyzrgb[base+2]);
   }
   
   public BufferedImage getRGBImage()
   {
      BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_4BYTE_ABGR);
      for(int h=0;h<height;h++)
         for(int w=0;w<width;w++)
            image.setRGB(w, h, Float.floatToRawIntBits(xyzrgb[4*(h*width+w)+3]));
      return image;
   }

   public void set(int w, int h, float x, float y, float z, int rgb)
   {
      int base=4*(h*width+w);
      xyzrgb[base+0]=x;
      xyzrgb[base+1]=y;
      xyzrgb[base+2]=z;
      xyzrgb[base+3]=Float.intBitsToFloat(rgb);
   }
   

}
