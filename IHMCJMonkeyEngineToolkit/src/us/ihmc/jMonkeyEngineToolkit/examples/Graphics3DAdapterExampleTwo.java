package us.ihmc.jMonkeyEngineToolkit.examples;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.ClassicCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.SimpleCameraTrackingAndDollyPositionHolder;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.jMonkeyEngineToolkit.utils.GraphicsDemoTools;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;
import us.ihmc.tools.thread.ThreadTools;

public class Graphics3DAdapterExampleTwo
{
   public void doExample(Graphics3DAdapter adapter)
   {
      Random random = new Random(1776L);
      
      Graphics3DNode node1 = new Graphics3DNode("node1", Graphics3DNodeType.JOINT);
      
      RigidBodyTransform transform1 = new RigidBodyTransform();
      transform1.setTranslation(new Vector3D(2.0, 0.0, 0.0));
      node1.setTransform(transform1);
      
      Graphics3DNode node2 = new Graphics3DNode("node2", Graphics3DNodeType.JOINT);
      Graphics3DNode rootNode = new Graphics3DNode("rootNode", Graphics3DNodeType.JOINT);
     
      Graphics3DObject object2 = GraphicsDemoTools.createCubeObject(0.6);
      Graphics3DObject object1 = GraphicsDemoTools.createRandomObject(random);
      
      node1.setGraphicsObject(object1);
      node2.setGraphicsObject(object2);
      
      rootNode.addChild(node1);
      rootNode.addChild(node2);
      
      adapter.addRootNode(rootNode);
      
      SelectedListener selectedListener = new SelectedListener()
      {
         public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3DReadOnly location, Point3DReadOnly cameraLocation,
               QuaternionReadOnly cameraRotation)
         {
            System.out.println("Selected " + graphics3dNode.getName() + " @ location " + location);                        
            
         }
      };
      
      adapter.addSelectedListener(selectedListener);
      node2.addSelectedListener(selectedListener);

      SimpleCameraTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder = new SimpleCameraTrackingAndDollyPositionHolder();
      ViewportAdapter camera = adapter.createNewViewport(null, false, false);
      ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(camera, cameraTrackAndDollyVariablesHolder, adapter);
      camera.setCameraController(classicCameraController);
      Canvas canvas = camera.getCanvas();
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);
      
      JFrame jFrame = new JFrame("Example Two");
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);
      
      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
      
      double rotation = 0.0;
      
      int count = 0;
      
      while (true)
      {
         rotation = rotation + 0.01;
         node2.getTransform().setRotationYaw(rotation);
         node2.getTransform().setTranslationToZero();
         
         count++;
         if (count > 200)
         {
            Graphics3DObject randomObject = GraphicsDemoTools.createRandomObject(random);
            node1.setGraphicsObject(randomObject);
            count = 0;
         }
         
         ThreadTools.sleep(1L);
      }
   }
}
