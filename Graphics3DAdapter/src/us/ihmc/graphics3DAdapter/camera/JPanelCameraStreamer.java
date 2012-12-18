package us.ihmc.graphics3DAdapter.camera;

import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.JPanel;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;


public class JPanelCameraStreamer extends JPanel implements CameraStreamer
{
   private static final long serialVersionUID = -6832977971630763132L;
   private BufferedImage bufferedImage;
   
   public JPanelCameraStreamer()
   {
      super();
   }
   
   public synchronized void updateImage(BufferedImage bufferedImage, Point3d cameraLocation, Quat4d cameraOrientation)
   {
      this.bufferedImage = bufferedImage;
      repaint();
   }

   protected synchronized void paintComponent(Graphics g)
   {
      if (bufferedImage != null)
      {
         g.drawImage(bufferedImage, 0, 0, this);
      }
   }

}