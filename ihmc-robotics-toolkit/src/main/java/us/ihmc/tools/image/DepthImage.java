package us.ihmc.tools.image;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;

import us.ihmc.euclid.tuple3D.Point3D32;

/**
 * Class to hold an image with depth data and all necessary transforms to convert to points
 * 
 * The (0,0) coordinate corresponds to the top left corner
 * 
 * The depths are stored as the distance from the camera in the z-axis. Note that this is not the distance between the camera and the point. 
 * 
 * @author Jesper Smith
 *
 */
public class DepthImage
{
   private final BufferedImage bufferedImage;
   private final float[] depths;

   private final int width;
   private final int height;

   private final byte[] imageArray;

   private float m00, m11;

   /**
    * Allocate a new depth image with a given width and height
    * 
    * @param width
    * @param height
    */
   public DepthImage(int width, int height)
   {
      this.width = width;
      this.height = height;

      bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
      depths = new float[width * height];

      WritableRaster wr = bufferedImage.getRaster();
      DataBufferByte db = (DataBufferByte) wr.getDataBuffer();
      imageArray = db.getData();

   }

   private int getDataIndex(int x, int y)
   {
      return ((height - y - 1) * width + x);
   }

   /**
    * Set the transform to calculate 3D coordinates from pixel coordinates.
    * 
    * The following formulates are used to calculate the X,Y,Z coordinates
    * 
    * x = (x'/width * 2 - 1) * m00 * depth
    * y = (y'/height * 2 - 1) * m11 * depth
    * z = depth
    * 
    * @param m00
    * @param m11
    */
   public void setTransform2D(float m00, float m11)
   {
      this.m00 = m00;
      this.m11 = m11;
   }

   /**
    * Set the color and depth of the point at pixel coordinates x, y
    * 
    * @param x
    * @param y
    * @param red
    * @param green
    * @param blue
    * @param depth
    */
   public void setPoint(int x, int y, byte red, byte green, byte blue, float depth)
   {
      int dataIndex = getDataIndex(x, y);
      int lowerHalfPtr = dataIndex * 3;
      depths[dataIndex] = depth;

      imageArray[lowerHalfPtr + 0] = blue;
      imageArray[lowerHalfPtr + 1] = green;
      imageArray[lowerHalfPtr + 2] = red;

   }

   /**
    * Get the depth of the point at pixel coordinates x,y
    * 
    * The depth measurement is -inf if an object was closer to the camera than the minimum distance
    * The depth measurement is +inf if and object was farther away from the camera than the maximum distance
    * 
    * 
    * @param x pixel coordinate
    * @param y pixel coordinate
    * 
    * @return Depth measurement
    */
   public float getDepth(int x, int y)
   {
      return depths[getDataIndex(x, y)];
   }

   /**
    * 
    * Get the point at pixel coordinates x,y and return the color
    * 
    * If there is no valid depth measurement at the this point (depth is -inf, +inf or NaN) then result is set to NaN for x,y and z.
    * 
    * @param x pixel coordinate
    * @param y pixel coordinate
    * @param result point to pack
    * @return Color packed in int as A-RGB, one byte per channel
    */
   public int getPoint(int x, int y, Point3D32 result)
   {
      int dataIndex = getDataIndex(x, y);
      int lowerHalfPtr = dataIndex * 3;

      if (result != null)
      {
         float depth = depths[dataIndex];
         if (Float.isInfinite(depth) || Float.isNaN(depth))
         {
            result.setToNaN();
         }
         else
         {
            result.setY(-1 * (((float) x / (float) width) * 2f - 1f) * m00);
            result.setZ((((float) y / (float) height) * 2f - 1f) * m11);
            result.setX(1);
            
            result.scale(depth);

         }
      }

      int a = 0xFF;
      int r = imageArray[lowerHalfPtr + 2];
      int g = imageArray[lowerHalfPtr + 1];
      int b = imageArray[lowerHalfPtr + 0];

      return (a << 24) + (r << 16) + (g << 8) + b;
   }

   /**
    * Get the color at pixel coordinates x,y
    * 
    * @param x pixel coordinate
    * @param y pixel coordinate
    *
    * @return Color packed in int as A-RGB, one byte per channel
    */
   public int getColor(int x, int y)
   {
      return getPoint(x, y, null);
   }
   
   /**
    * Get the buffered image for this depth image
    * 
    * @return Buffered image
    */
   public BufferedImage getBufferedImage()
   {
      return bufferedImage;
   }
}
