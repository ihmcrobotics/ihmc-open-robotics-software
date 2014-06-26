package us.ihmc.SdfLoader;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFGeometry.HeightMap;

public class SDFHeightMap implements us.ihmc.graphics3DAdapter.HeightMap
{
   
   double xMin, xMax, yMin, yMax, zOffset;
   double scale;
   
   int width, height;
   byte[] data;

   private final Vector3d offset;
   
   public SDFHeightMap(String URI, HeightMap heightMap)
   {
      Vector3d size = SDFConversionsHelper.stringToVector3d(heightMap.getSize());
      offset = SDFConversionsHelper.stringToVector3d(heightMap.getPos());
      try
      {
         BufferedImage img = ImageIO.read(new File(URI));
         BufferedImage convertedImg = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_BYTE_GRAY);
         convertedImg.getGraphics().drawImage(img, 0, 0, img.getWidth(), img.getHeight(), 0, img.getHeight(), img.getWidth(), 0, null);
         
         data = (( DataBufferByte) convertedImg.getData().getDataBuffer() ).getData();
         width = img.getWidth();
         height = img.getHeight();
         
         
         
         xMin = -size.getX()/2.0;
         xMax = size.getX()/2.0;
         
         yMin = -size.getY()/2.0;
         yMax = size.getY()/2.0;
         
         scale = size.getZ()/255.0;
         
         
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public double heightAt(double x, double y, double z)
   {
      double xFactor = (x - xMin) / (xMax - xMin);
      double yFactor = (y - yMin) / (yMax - yMin);
      
      int xPoint = (int) Math.round(xFactor * ((double)width));
      int yPoint = (int) Math.round(yFactor * ((double)height));
      
      int index = xPoint + yPoint * width;
      if(index >= data.length)
      {
         return 0.0;
      }
      return ((double) (data[index] & 0xFF)) * scale;
      
   }
   
   public Vector3d getOffset()
   {
      return offset;
   }

   public double getXMin()
   {
      return xMin;
   }

   public double getXMax()
   {
      return xMax;
   }

   public double getYMin()
   {
      return yMin;
   }

   public double getYMax()
   {
      return yMax;
   }

}
