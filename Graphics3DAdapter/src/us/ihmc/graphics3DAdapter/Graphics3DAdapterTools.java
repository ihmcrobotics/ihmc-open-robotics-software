package us.ihmc.graphics3DAdapter;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.graphics3DAdapter.camera.CameraTrackingAndDollyPositionHolder;
import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.camera.SimpleCameraTrackingAndDollyPositionHolder;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;

public class Graphics3DAdapterTools
{
   
   public static void createNewWindow(Graphics3DAdapter graphics3DAdapter, String title, int width, int height)
   {
      ViewportAdapter viewportAdapter = graphics3DAdapter.createNewViewport(null, false);
      CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder = new SimpleCameraTrackingAndDollyPositionHolder();
      ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(viewportAdapter, cameraTrackingAndDollyPositionHolder, graphics3DAdapter);
      viewportAdapter.setCameraController(classicCameraController);
      Canvas canvas = viewportAdapter.getCanvas();
      
      Graphics3DAdapterTools.createNewWindow(canvas, title, width, height);
   }
   
   public static void createNewWindow(Canvas canvas, String title, int width, int height)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);

      JFrame jFrame = new JFrame(title);
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);

      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
   }
}
