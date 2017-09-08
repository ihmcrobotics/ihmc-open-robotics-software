package us.ihmc.jMonkeyEngineToolkit.jme.examples;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;
import java.awt.Dimension;
import java.util.concurrent.Callable;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.camera.ClassicCameraController;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer.RenderType;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEViewportAdapter;
import us.ihmc.jMonkeyEngineToolkit.utils.GraphicsDemoTools.PanBackAndForthTrackingAndDollyPositionHolder;

public class TransformRayDemo
{
   RigidBodyTransform tInit = new RigidBodyTransform();
   RigidBodyTransform tAct = new RigidBodyTransform();

   public static void main(String[] args)
   {
      new TransformRayDemo();
   }

   public TransformRayDemo()
   {
      final Graphics3DObject graphic = new Graphics3DObject();
      incrementalRotation.setRotationRollAndZeroTranslation(-Math.PI / 16);
      graphic.addCylinder(5.0, 0.2, YoAppearance.Red());

      final Graphics3DNode rayNode = new Graphics3DNode("laserRay", Graphics3DNodeType.VISUALIZATION);
      tInit.setRotationPitchAndZeroTranslation(Math.PI / 4.0);
      // tInit.setToRollMatrix(Math.PI / 4.0);
      rayNode.setTransform(tInit);
      rayNode.setGraphicsObject(graphic);

      final JMERenderer renderer = new JMERenderer(RenderType.AWTPANELS);
      renderer.setupSky();

      renderer.addRootNode(rayNode);

      PanBackAndForthTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder = new PanBackAndForthTrackingAndDollyPositionHolder(20.0, 20.0, 20.0);
      JMEViewportAdapter viewportAdapter = (JMEViewportAdapter) renderer.createNewViewport(null, false, false);
      ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(viewportAdapter,
                                                           cameraTrackAndDollyVariablesHolder, renderer);
      viewportAdapter.setCameraController(classicCameraController);

      Canvas canvas = viewportAdapter.getCanvas();
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);
      panel.setPreferredSize(new Dimension(1800, 1080));

      JFrame jFrame = new JFrame("Example One");
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);

      jFrame.pack();
      jFrame.setLocationRelativeTo(null);
      jFrame.setVisible(true);

      new Thread(new Runnable()
      {
         public void run()
         {
            while (true)
            {
               renderer.enqueue(new Callable<Object>()
               {
                  public Object call() throws Exception
                  {
                     rayNode.setTransform(generateTransform(rayNode.getTransform()));

//
//
//
//                   Transform3D rotatorZ = new Transform3D();
////                 rotator.setRotation(new AxisAngle4d(0, 0, 1, Math.PI / 100));
//                   rotatorZ.rotZ(Math.PI / 16);
//
////                 Transform3D rotatorX = new Transform3D(rayNode.getTransform());
////                 rotatorX.setToRollMatrix(Math.PI / 16);
//
//                   // rotatorZ.mul(rotatorX);
//                   tAct.mul(rotatorZ);
//                   rotatorZ.mul(tInit, tAct);
//                   rayNode.setTransform(rotatorZ);
//                   // rayNode.getTransform().mul(rotatorX);

                     return null;
                  }
               });

               try
               {
                  Thread.sleep(50);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }
            }

         }
      }).start();
   }

   RigidBodyTransform incrementalRotation = new RigidBodyTransform();

   public AffineTransform generateTransform(AffineTransform init)
   {
      AffineTransform rotated = new AffineTransform(init);
      rotated.multiply(incrementalRotation);

      return rotated;
   }
}
