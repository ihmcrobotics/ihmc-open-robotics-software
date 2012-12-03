package us.ihmc.darpaRoboticsChallenge.userInterface;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.Serializable;

import javax.imageio.ImageIO;

import sun.awt.image.ImageFormatException;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.sun.image.codec.jpeg.JPEGCodec;
import com.sun.image.codec.jpeg.JPEGEncodeParam;
import com.sun.image.codec.jpeg.JPEGImageEncoder;

public class ScreenCapture implements Serializable
{
   /**
    *
    */
   private static final long serialVersionUID = 1L;
   private byte[] _bytesOut = null;
   private int _height, _width;
   private Vector3f location;
   private Quaternion rotation;

   public Vector3f getLocation()
   {
      return location;
   }



   public void setLocation(Vector3f location)
   {
      this.location = location;
   }



   public Quaternion getRotation()
   {
      return rotation;
   }




   public ScreenCapture(BufferedImage bi, Vector3f location, Quaternion rotation)
   {
      _height = bi.getHeight();
      _width = bi.getWidth();

      try
      {
         _bytesOut = bufferedImageToByteArray(bi);
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

      System.out.println("size " + _bytesOut.length);

      // bi.getRGB(0, 0, _width, _height, _bytesOut, 0, _width);
   }

   public static byte[] bufferedImageToByteArray(BufferedImage img) throws ImageFormatException, IOException
   {
      ByteArrayOutputStream os = new ByteArrayOutputStream();

      JPEGImageEncoder encoder = JPEGCodec.createJPEGEncoder(os);
      JPEGEncodeParam param = encoder.getDefaultJPEGEncodeParam(img);
      param.setQuality(0.1f, true);
      encoder.setJPEGEncodeParam(param);
      encoder.encode(img);

      return os.toByteArray();
   }

   public BufferedImage byteArrayToBufferedImage(byte[] bytes)
   {
      InputStream in = new ByteArrayInputStream(bytes);
      BufferedImage bImageFromConvert = null;
      try
      {
         bImageFromConvert = ImageIO.read(in);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return bImageFromConvert;
   }

   public BufferedImage getImage()
   {
//    BufferedImage bi = new BufferedImage(_width, _height, BufferedImage.TYPE_4BYTE_ABGR);
//    bi.setRGB(0, 0, _width, _height, _bytesOut, 0, _width);

      return byteArrayToBufferedImage(_bytesOut);
   }
}
