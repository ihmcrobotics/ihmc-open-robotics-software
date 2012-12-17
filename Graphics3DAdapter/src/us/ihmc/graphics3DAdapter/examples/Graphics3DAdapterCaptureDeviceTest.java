package us.ihmc.graphics3DAdapter.examples;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Container;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.camera.CameraStreamer;
import us.ihmc.graphics3DAdapter.camera.CaptureDevice;
import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.camera.SimpleCameraTrackingAndDollyPositionHolder;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.ThreadTools;

public class Graphics3DAdapterCaptureDeviceTest
{
   public void doExample(Graphics3DAdapter graphics3DAdapter)
   {
      Graphics3DNode teapotAndSphereNode = new Graphics3DNode("teaPot", Graphics3DNodeType.JOINT);
      Graphics3DObject teapotObject = new Graphics3DObject();
      teapotObject.addTeaPot(YoAppearance.Red());

      teapotAndSphereNode.setGraphicsObject(teapotObject);
      graphics3DAdapter.addRootNode(teapotAndSphereNode);

      PanBackAndForthTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder = new PanBackAndForthTrackingAndDollyPositionHolder(0.0, 2.0, 0.2);
      
      ViewportAdapter viewportAdapter = graphics3DAdapter.createNewViewport(null);
      ClassicCameraController classicCameraController = new ClassicCameraController(viewportAdapter, cameraTrackAndDollyVariablesHolder);
      classicCameraController.setTracking(true, true, false, false);

      viewportAdapter.setCameraController(classicCameraController);
      viewportAdapter.setupOffscreenView(800, 600);


      CaptureDevice captureDevice = viewportAdapter.getCaptureDevice();
      VideoCapture videoCapture = new VideoCapture();
      captureDevice.streamTo(videoCapture);

      createNewWindow(videoCapture);
      
      while (true)
      {
         cameraTrackAndDollyVariablesHolder.run();
         try
         {
            Thread.sleep(10L);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }

   }
   
   

   public class VideoCapture extends JPanel implements CameraStreamer
   {
      private static final long serialVersionUID = -6832977971630763132L;
      private BufferedImage bufferedImage;
      
      public VideoCapture()
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
   
   private class PanBackAndForthTrackingAndDollyPositionHolder extends SimpleCameraTrackingAndDollyPositionHolder implements Runnable
   {
      private long startTime = System.currentTimeMillis();
      private final double panXOffset, panXAmplitude, panXFrequency;
      
      public PanBackAndForthTrackingAndDollyPositionHolder(double panXOffset, double panXAmplitude, double panXFrequency)
      {
         this.panXOffset = panXOffset;
         this.panXAmplitude = panXAmplitude;
         this.panXFrequency = panXFrequency;
         
         Thread thread = new Thread(this);
         thread.start();
      }

      public void run()
      {
         while(true)
         {
            long currentTime = System.currentTimeMillis(); 
            double time = (currentTime - startTime) * 0.001;
            
            double cameraTrackingX = panXOffset + panXAmplitude * Math.sin(2.0 * Math.PI * panXFrequency * time);
            this.setTrackingX(cameraTrackingX);
            
            ThreadTools.sleep(100L);

         }
         
      }
      
      
   }
   
   public void createNewWindow(Component canvas)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);
      
      JFrame jFrame = new JFrame("Example One");
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);
      
      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
   }
}
