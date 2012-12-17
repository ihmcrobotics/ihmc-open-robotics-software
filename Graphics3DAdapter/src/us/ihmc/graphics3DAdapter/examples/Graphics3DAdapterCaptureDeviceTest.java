package us.ihmc.graphics3DAdapter.examples;

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
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.ThreadTools;

public class Graphics3DAdapterCaptureDeviceTest
{
   public void doExample(Graphics3DAdapter graphics3DAdapter)
   {
      Graphics3DNode teapotAndSphereNode = new Graphics3DNode("teaPot", Graphics3DNodeType.JOINT);
      Graphics3DObject teapotObject = new Graphics3DObject();
      teapotAndSphereNode.setGraphicsObject(teapotObject);
      graphics3DAdapter.addRootNode(teapotAndSphereNode);

      ViewportAdapter viewportAdapter = graphics3DAdapter.createNewViewport(null);
      PanBackAndForthTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder = new PanBackAndForthTrackingAndDollyPositionHolder(0.0, 2.0, 0.2);
      viewportAdapter.setCameraController(ClassicCameraController.createClassicCameraControllerAndAddListeners(viewportAdapter, cameraTrackAndDollyVariablesHolder, graphics3DAdapter));
      viewportAdapter.setupOffscreenView(800, 600);

      CaptureDevice captureDevice = viewportAdapter.getCaptureDevice();
      VideoCapture videoCapture = new VideoCapture();
      captureDevice.streamTo(videoCapture);

      JFrame f = new JFrame();
      f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      f.add(videoCapture);
      f.setSize(800, 600);
      f.setLocation(200, 200);
      f.setVisible(true);

      while (true)
      {
         cameraTrackAndDollyVariablesHolder.run();
         try
         {
            Thread.sleep(100L);
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

      public synchronized void updateImage(BufferedImage bufferedImage, Point3d cameraLocation, Quat4d cameraOrientation)
      {
         this.bufferedImage = bufferedImage;
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
}
