package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.Serializable;

import javax.imageio.ImageIO;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;


public class ScreenCapture implements Serializable
{
   private static final long serialVersionUID = 4267642643460511978L;
   private static final boolean isOpenJDK = System.getProperty("java.vm.name").indexOf("OpenJDK") != -1;
   static
   {
      if(isOpenJDK)
      {
         System.err.println("OpenJDK doesn't ship with JPEG libraries. Streaming using PNG images.");
      }
   }
   private byte[] bytesOut = null;
   private final int height, width;
   private final Point3D location;
   private final Quaternion rotation;
   private float fov;

   public ScreenCapture(BufferedImage bufferedImage, Point3D location, Quaternion rotation, float fov)
   {
      this.height = bufferedImage.getHeight();
      this.width = bufferedImage.getWidth();

      try
      {
         bytesOut = bufferedImageToByteArray(bufferedImage);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      this.rotation = rotation;
      this.location = location;
      this.fov = fov;

      // bi.getRGB(0, 0, _width, _height, _bytesOut, 0, _width);
   }

   public Point3D getLocation()
   {
      return location;
   }

   public Quaternion getRotation()
   {
      return rotation;
   }
   
   public float getFov()
   {
      return fov;
   }
   
   public static byte[] bufferedImageToByteArray(BufferedImage image) throws IOException
   {
      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();

      if(isOpenJDK)
      {
         ImageIO.write(image, "png", outputStream);
      }
      else
      {
         ImageIO.write(image, "jpeg", outputStream);
      }

      // JPEGImageEncoder encoder = JPEGCodec.createJPEGEncoder(os);
      // JPEGEncodeParam param = encoder.getDefaultJPEGEncodeParam(img);
      // param.setQuality(0.1f, true);
      // encoder.setJPEGEncodeParam(param);
      // encoder.encode(img);

      return outputStream.toByteArray();
   }

   public static BufferedImage byteArrayToBufferedImage(byte[] bytes)
   {
      if (bytes == null) return null;
      
      InputStream inputStream = new ByteArrayInputStream(bytes);
      BufferedImage bufferedImageFromConvert = null;
      try
      {
         bufferedImageFromConvert = ImageIO.read(inputStream);
      }
      catch (IOException e)
      {
         return null;
//         e.printStackTrace();
      }

      return bufferedImageFromConvert;
   }

   public BufferedImage getImage()
   {
//    BufferedImage bi = new BufferedImage(_width, _height, BufferedImage.TYPE_4BYTE_ABGR);
//    bi.setRGB(0, 0, _width, _height, _bytesOut, 0, _width);

      return byteArrayToBufferedImage(bytesOut);
   }

   public int getHeight()
   {
      return height;
   }

   public int getWidth()
   {
      return width;
   }

}
