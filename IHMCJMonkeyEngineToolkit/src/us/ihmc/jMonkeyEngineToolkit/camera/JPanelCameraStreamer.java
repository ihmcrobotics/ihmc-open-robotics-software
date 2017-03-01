package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.JPanel;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;


public class JPanelCameraStreamer extends JPanel implements CameraStreamer
{
   private static final long serialVersionUID = -6832977971630763132L;
   private BufferedImage bufferedImage;
   
   public JPanelCameraStreamer()
   {
      super();
   }

   public synchronized void updateImage(BufferedImage bufferedImage)
   {
      this.bufferedImage = bufferedImage;
      repaint();
   }

   public void setIntrinsic(IntrinsicParameters param)
   {

   }

   public synchronized void updateImage(BufferedImage bufferedImage, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParamaters)
   {
      updateImage(bufferedImage);
   }
   
   public synchronized void updateImage(BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, double fov)
   {
      updateImage(bufferedImage);
   }

   protected synchronized void paintComponent(Graphics g)
   {
      if (bufferedImage != null)
      {
         g.drawImage(bufferedImage, 0, 0, this);
      }
   }

   public void createAndDisplayInNewWindow(String name)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", this);

      JFrame jFrame = new JFrame(name);
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);

      jFrame.pack();
      jFrame.setLocationByPlatform(true); 
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
   }


   public Point3D getCameraPosition()
   {
      // TODO Auto-generated method stub
      return null;
   }


   public Quaternion getCameraOrientation()
   {
      // TODO Auto-generated method stub
      return null;
   }


   public double getFieldOfView()
   {
      // TODO Auto-generated method stub
      return 0;
   }


   public boolean isReadyForNewData()
   {
      return true;
   }

   public long getTimeStamp()
   {
      return 0;
   }

}