package us.ihmc.jMonkeyEngineToolkit.examples;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.ClassicCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.SimpleCameraTrackingAndDollyPositionHolder;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.geometry.Transform3d;

public class Graphics3DAdapterExampleOne
{   

   public Boolean doExample(Graphics3DAdapter graphics3DAdapter)
   {
      Graphics3DNode teapotAndSphereNode = new Graphics3DNode("teaPot", Graphics3DNodeType.JOINT);
      Graphics3DObject teapotObject = new Graphics3DObject();
      teapotObject.setChangeable(true);
      Graphics3DInstruction teapotAppearanceHolder = teapotObject.addTeaPot(YoAppearance.Red());
      teapotAndSphereNode.setGraphicsObject(teapotObject);
      graphics3DAdapter.addRootNode(teapotAndSphereNode);
      BlinkRunnable blinker = new BlinkRunnable(teapotAppearanceHolder);
      
      Graphics3DNode box = new Graphics3DNode("box", Graphics3DNodeType.JOINT);
      Graphics3DObject boxGraphics = new Graphics3DObject();
      boxGraphics.addCube(1.0, 1.0, 1.0, YoAppearance.Green());
      box.setGraphicsObject(boxGraphics);
      graphics3DAdapter.addRootNode(box);

      PanBackAndForthTrackingAndDollyPositionHolder cameraTrackAndDollyVariablesHolder = new PanBackAndForthTrackingAndDollyPositionHolder(0.0, 2.0, 0.2);
      
      ViewportAdapter viewportAdapter = graphics3DAdapter.createNewViewport(null, false, false);
      ClassicCameraController classicCameraController = ClassicCameraController.createClassicCameraControllerAndAddListeners(viewportAdapter, cameraTrackAndDollyVariablesHolder, graphics3DAdapter);
      viewportAdapter.setCameraController(classicCameraController);
      Canvas canvas = viewportAdapter.getCanvas();
      createNewWindow(canvas);
      
      ViewportAdapter secondCamera = graphics3DAdapter.createNewViewport(null, false, false);
      ClassicCameraController secondController = ClassicCameraController.createClassicCameraControllerAndAddListeners(secondCamera, cameraTrackAndDollyVariablesHolder, graphics3DAdapter);
      secondCamera.setCameraController(secondController);
      createNewWindow(secondCamera.getCanvas());
      
      classicCameraController.setTracking(true, true, false, false);
      
      SelectedListener selectedListener = new SelectedListener()
      {

         public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyHolder, Point3d location, Point3d cameraLocation,
               Quat4d cameraRotation)
         {
        	 if(graphics3dNode != null)
        	 {
        		 System.out.println("Selected " + graphics3dNode.getName() + " @ location " + location);                                		 
        	 }
            
         }
      };
      
      
      graphics3DAdapter.addSelectedListener(selectedListener);
      box.addSelectedListener(selectedListener);
      
      RotateAndScaleNodeRunnable rotator = new RotateAndScaleNodeRunnable(teapotAndSphereNode);

      ArrayList<Runnable> runnables = new ArrayList<Runnable>();
      runnables.add(rotator);
      runnables.add(blinker);
      
      for (int i = 0; i < 600; i++)
      {
         for (Runnable runnable : runnables)
         {
            runnable.run();
         }

         try
         {
            Thread.sleep(10L);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
      
      return true;
   }
   
   
   
   public void createWindow(Canvas canvas1, Canvas canvas2)
   {
      JPanel panel = new JPanel(new FlowLayout());
      panel.add("Center", canvas1);
      panel.add("East", canvas2);
      canvas1.setPreferredSize(new Dimension(390,600));
      canvas2.setPreferredSize(new Dimension(390,600));
      
      JFrame jFrame = new JFrame("Example One");
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add(panel);
      
      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
   }

   public void createNewWindow(Canvas canvas)
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
   
   public static Graphics3DObject createSphereObject(double radius)
   {
      Graphics3DObject sphere = new Graphics3DObject();
      sphere.addSphere(radius, YoAppearance.Green());

      return sphere;
   }
   
   public static Graphics3DObject createCylinderObject(double radius)
   {
      Graphics3DObject cylinder = new Graphics3DObject();
      double height = 1.0;
      cylinder.addCylinder(height, radius, YoAppearance.Pink());

      return cylinder;
   }
   
   public static Graphics3DObject createCubeObject(double lengthWidthHeight)
   {
      Graphics3DObject cube = new Graphics3DObject();
      cube.addCube(lengthWidthHeight, lengthWidthHeight, lengthWidthHeight, YoAppearance.Red());

      return cube;
   }
   
   public static Graphics3DObject createRandomObject(Random random)
   {
      int selection = random.nextInt(3);
      
      switch (selection)
      {
      case 0:
      {
         return createCubeObject(random.nextDouble());
      }
      case 1:
      {
         return createSphereObject(random.nextDouble() * 0.5);
      }
      case 2:
      {
         return createCylinderObject(random.nextDouble() * 0.5);
      }
      default:
      {
         throw new RuntimeException("Should not get here");
      }
      }
   }

   
   private class BlinkRunnable implements Runnable
   {
      private final Graphics3DInstruction instruction;
      private double transparency = 0.0;
      
      public BlinkRunnable(Graphics3DInstruction instruction)
      {
         this.instruction = instruction;
      }

      public void run()
      {
         transparency += 0.01;
         if (transparency > 1.0) transparency = 0.0;
         
         Color3f color = new Color3f((float) Math.random(), (float) Math.random(), (float) Math.random());
         YoAppearanceRGBColor appearance = new YoAppearanceRGBColor(color, 0.0);
         appearance.setTransparency(transparency);
         instruction.setAppearance(appearance);
      }
   
   }
   
   private class RotateAndScaleNodeRunnable implements Runnable
   {
      private final Graphics3DNode node;
      private double rotation = 0.0;
      private double scale = 1.0;
      private double translation = 0.0;
      private boolean scalingDown = true;
      
      public RotateAndScaleNodeRunnable(Graphics3DNode node)
      {
         this.node = node;
      }

      public void run()
      {
         rotation += 0.01;
         translation += 0.01;
         
         if (scalingDown)
         {
            scale = scale * 0.99;
            if (scale < 0.1) scalingDown = false;
         }
         else
         {
            scale = scale * 1.01;
            if (scale > 2.0) scalingDown = true;
         }
         
         Transform3d transform = new Transform3d();
         transform.setRotationEulerAndZeroTranslation(Math.PI/2.0, 0.0, rotation);
         transform.setTranslation(new Vector3d(translation, 0.0, 0.0));
         transform.setScale(scale);
         node.setTransform(transform);
      }
   }
   
   private class PanBackAndForthTrackingAndDollyPositionHolder extends SimpleCameraTrackingAndDollyPositionHolder implements Runnable
   {
      private final long startTime = System.currentTimeMillis();
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
