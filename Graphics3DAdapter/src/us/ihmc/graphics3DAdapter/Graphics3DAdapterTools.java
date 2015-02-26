package us.ihmc.graphics3DAdapter;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;
import java.awt.Dimension;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.camera.CameraController;
import us.ihmc.graphics3DAdapter.camera.CameraTrackingAndDollyPositionHolder;
import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.camera.SimpleCameraTrackingAndDollyPositionHolder;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;

public class Graphics3DAdapterTools
{
   public static ViewportAdapter createViewport(Graphics3DAdapter graphics3DAdapter)
   {
      return graphics3DAdapter.createNewViewport(null, false, false);
   }

   public static ClassicCameraController createCameraController(Graphics3DAdapter graphics3DAdapter, ViewportAdapter viewportAdapter,
           Vector3d initialCameraTranslation)
   {
      CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder = new SimpleCameraTrackingAndDollyPositionHolder();
      ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(viewportAdapter,
                                                           cameraTrackingAndDollyPositionHolder, graphics3DAdapter);
      classicCameraController.setCameraPosition(initialCameraTranslation.x, initialCameraTranslation.y, initialCameraTranslation.z);

      return classicCameraController;
   }

   public static ClassicCameraController createNewWindow(Graphics3DAdapter graphics3DAdapter, ViewportAdapter viewportAdapter, String title, int width,
           int height, Vector3d initialCameraTranslation)
   {
      ClassicCameraController classicCameraController = createCameraController(graphics3DAdapter, viewportAdapter, initialCameraTranslation);

      viewportAdapter.setCameraController(classicCameraController);

      createNewWindow(viewportAdapter.getCanvas(), title, width, height);

      return classicCameraController;
   }

   public static JFrame createNewWindow(ViewportAdapter viewportAdapter, String title, int width, int height, CameraController cameraController)
   {
      viewportAdapter.setCameraController(cameraController);

      return createNewWindow(viewportAdapter.getCanvas(), title, width, height);
   }

   public static ClassicCameraController createNewWindow(Graphics3DAdapter graphics3DAdapter, String title, int width, int height,
           Vector3d initialCameraTranslation)
   {
      return createNewWindow(graphics3DAdapter, createViewport(graphics3DAdapter), title, width, height, initialCameraTranslation);
   }

   public static ClassicCameraController createNewWindow(Graphics3DAdapter graphics3DAdapter, String title, int width, int height)
   {
      return createNewWindow(graphics3DAdapter, title, width, height,
                             new Vector3d(ClassicCameraController.CAMERA_START_X, ClassicCameraController.CAMERA_START_Y,
                                ClassicCameraController.CAMERA_START_Z));
   }

   public static JFrame createNewWindow(Canvas canvas, String title, int width, int height)
   {
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);
      panel.setPreferredSize(new Dimension(width, height));

      JFrame jFrame = new JFrame(title);
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);

      jFrame.setIconImage(new ImageIcon("resources/running-man-32x32.png").getImage());
      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setLocationRelativeTo(null);

      return jFrame;
   }
}
