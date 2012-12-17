package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.Serializable;

import javax.imageio.ImageIO;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import sun.awt.image.ImageFormatException;


public class ScreenCapture implements Serializable
{
   private static final long serialVersionUID = 4267642643460511978L;
   private byte[] bytesOut = null;
   private final int height, width;
   private final Vector3f location;
   private final Quat4f rotation;

   public ScreenCapture(BufferedImage bufferedImage, Vector3f location, Quat4f rotation)
   {
      this.height = bufferedImage.getHeight();
      this.width = bufferedImage.getWidth();

      try
      {
         bytesOut = bufferedImageToByteArray(bufferedImage);
      }
      catch (ImageFormatException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      this.rotation = rotation;
      this.location = location;


      // bi.getRGB(0, 0, _width, _height, _bytesOut, 0, _width);
   }

   public Vector3f getLocation()
   {
      return location;
   }

   public Quat4f getRotation()
   {
      return rotation;
   }
   
   public static byte[] bufferedImageToByteArray(BufferedImage image) throws ImageFormatException, IOException
   {
      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();

      ImageIO.write(image, "jpeg", outputStream);

      // JPEGImageEncoder encoder = JPEGCodec.createJPEGEncoder(os);
      // JPEGEncodeParam param = encoder.getDefaultJPEGEncodeParam(img);
      // param.setQuality(0.1f, true);
      // encoder.setJPEGEncodeParam(param);
      // encoder.encode(img);

      return outputStream.toByteArray();
   }

   public static BufferedImage byteArrayToBufferedImage(byte[] bytes)
   {
      InputStream inputStream = new ByteArrayInputStream(bytes);
      BufferedImage bufferedImageFromConvert = null;
      try
      {
         bufferedImageFromConvert = ImageIO.read(inputStream);
      }
      catch (IOException e)
      {
         e.printStackTrace();
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
