package us.ihmc.graphics3DAdapter.camera;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;


public class JPanelCameraStreamer extends JPanel implements CameraViewer, VideoStreamer, CameraStreamer
{
   private static final long serialVersionUID = -6832977971630763132L;
   private BufferedImage bufferedImage;
   
   public JPanelCameraStreamer()
   {
      super();
   }

   
   public synchronized void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, double fov)
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

   public void createAndDisplayInNewWindow(String name, int xLocation, int yLocation)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", this);

      JFrame jFrame = new JFrame(name);
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);

      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
      jFrame.setLocation(xLocation, yLocation); 
   }


   public Point3d getCameraPosition()
   {
      // TODO Auto-generated method stub
      return null;
   }


   public Quat4d getCameraOrientation()
   {
      // TODO Auto-generated method stub
      return null;
   }


   public double getFieldOfView()
   {
      // TODO Auto-generated method stub
      return 0;
   }

}