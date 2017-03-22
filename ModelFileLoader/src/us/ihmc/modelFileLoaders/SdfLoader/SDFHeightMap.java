package us.ihmc.modelFileLoaders.SdfLoader;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.IOException;

import javax.imageio.ImageIO;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry.HeightMap;

public class SDFHeightMap implements us.ihmc.graphicsDescription.HeightMap
{
   double xMin, xMax, yMin, yMax, zOffset;
   private BoundingBox3D boundingBox;

   double scale;
   
   int width, height;
   byte[] data;

   private final Vector3D offset;
   
   public SDFHeightMap(String resourceId, HeightMap heightMap)
   {
      Vector3D size = ModelFileLoaderConversionsHelper.stringToVector3d(heightMap.getSize());
      offset = ModelFileLoaderConversionsHelper.stringToVector3d(heightMap.getPos());
      try
      {
         BufferedImage img = ImageIO.read(getClass().getClassLoader().getResourceAsStream(resourceId));
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
         
         double zMin = Double.NEGATIVE_INFINITY;
         double zMax = Double.POSITIVE_INFINITY;
         boundingBox = new BoundingBox3D(xMin, yMin, zMin, xMax, yMax, zMax);
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
   
   public Vector3D getOffset()
   {
      return offset;
   }

   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

}
