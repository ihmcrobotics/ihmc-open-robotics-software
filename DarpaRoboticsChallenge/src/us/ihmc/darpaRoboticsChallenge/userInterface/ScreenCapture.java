package us.ihmc.darpaRoboticsChallenge.userInterface;

import java.awt.image.BufferedImage;
import java.io.Serializable;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class ScreenCapture implements Serializable
{
   /**
    *
    */
   private static final long serialVersionUID = 1L;
   private int[] _bytesOut = null;
   private int _height, _width;
   private Vector3f location;

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




   private Quaternion rotation;

   public ScreenCapture(BufferedImage bi, Vector3f location, Quaternion rotation)
   {
      _height = bi.getHeight();
      _width = bi.getWidth();
      _bytesOut = new int[_width * _height];
      this.rotation = rotation;
      this.location = location;

      bi.getRGB(0, 0, _width, _height, _bytesOut, 0, _width);
   }



   public BufferedImage getImage()
   {
      BufferedImage bi = new BufferedImage(_width, _height, BufferedImage.TYPE_4BYTE_ABGR);
      bi.setRGB(0, 0, _width, _height, _bytesOut, 0, _width);

      return bi;
   }
}
