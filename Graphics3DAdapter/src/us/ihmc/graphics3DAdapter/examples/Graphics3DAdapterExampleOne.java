package us.ihmc.graphics3DAdapter.examples;

import java.awt.BorderLayout;
import java.awt.Canvas;
import java.awt.Container;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Color3f;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.NodeType;
import us.ihmc.graphics3DAdapter.SelectedListener;
import us.ihmc.graphics3DAdapter.graphics.LinkGraphics;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsInstruction;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public class Graphics3DAdapterExampleOne
{
   public void doExampleOne(Graphics3DAdapter adapter)
   {
      Graphics3DNode teapotAndSphereNode = new Graphics3DNode("teaPot", NodeType.JOINT);
      LinkGraphics teapotObject = createTeapotObject();
      LinkGraphicsInstruction sphereAppearanceHolder = teapotObject.addSphere(2.0, YoAppearance.Red());

      teapotAndSphereNode.setGraphicsObject(teapotObject);
      adapter.addRootNode(teapotAndSphereNode);

      Canvas canvas = adapter.getDefaultCamera().getCanvas();
      JPanel panel = new JPanel(new BorderLayout());
      panel.add("Center", canvas);
      
      JFrame jFrame = new JFrame("Example One");
      Container contentPane = jFrame.getContentPane();
      contentPane.setLayout(new BorderLayout());
      contentPane.add("Center", panel);
      
      jFrame.pack();
      jFrame.setVisible(true);
      jFrame.setSize(800, 600);
      
      
      SelectedListener selectedListener = new SelectedListener()
      {
         
         public void selected(Graphics3DNode graphics3dNode, String modifierKey)
         {
            System.out.println("Selected " + graphics3dNode.getName());
         }
      };
      
      
      adapter.addSelectedListener(selectedListener);
      teapotAndSphereNode.addSelectedListener(selectedListener);
      
      RotateAndScaleNodeRunnable rotator = new RotateAndScaleNodeRunnable(teapotAndSphereNode);
      BlinkRunnable blinker = new BlinkRunnable(sphereAppearanceHolder);
     

      ArrayList<Runnable> runnables = new ArrayList<Runnable>();
//      runnables.add(rotator);
//      runnables.add(blinker);

      while (true)
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
   }

   private LinkGraphics createTeapotObject()
   {
	      //teapot = assetManager.loadModel("Models/Teapot/Teapot.mesh.xml");
      LinkGraphics teapotObject = new LinkGraphics();
//      teapotObject.addModelFile("Models/Teapot/Teapot.mesh.xml");
      teapotObject.translate(0.0, 1.0, 1.0);
      teapotObject.rotate(Math.PI / 4.0, LinkGraphics.X);
      teapotObject.addEllipsoid(2.0, 2.0, 1.5);
      teapotObject.translate(0.0, 2.0, 1.0);
      teapotObject.rotate(Math.PI / 4.0, LinkGraphics.X);

      return teapotObject;
   }

   
   private class BlinkRunnable implements Runnable
   {
      private final LinkGraphicsInstruction instruction;
      private double transparency = 0.0;
      
      public BlinkRunnable(LinkGraphicsInstruction instruction)
      {
         this.instruction = instruction;
      }

      public void run()
      {
         transparency += 0.01;
         if (transparency > 1.0) transparency = 0.0;
         
         Color3f color = new Color3f((float) Math.random(), (float) Math.random(), (float) Math.random());
         YoAppearanceRGBColor appearance = new YoAppearanceRGBColor(color);
         appearance.setTransparancy(transparency);
         instruction.setAppearance(appearance);
      }
   
   }
   
   private class RotateAndScaleNodeRunnable implements Runnable
   {
      private final Graphics3DNode node;
      private double rotation = 0.0;
      private double scale = 1.0;
      private boolean scalingDown = true;
      
      public RotateAndScaleNodeRunnable(Graphics3DNode node)
      {
         this.node = node;
      }

      public void run()
      {
         rotation += 0.01;
         
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
         
         Transform3D transform = new Transform3D();
         transform.rotZ(rotation);
         transform.setScale(scale);
         node.setTransform(transform);
      }
   }


   abstract class LinkGraphicsInstructionModifyingRunnable implements Runnable
   {
      LinkGraphicsInstruction instruction;

      abstract public void run();

      public void setLinkGraphicsInstruction(LinkGraphicsInstruction instruction)
      {
         this.instruction = instruction;
      }
   }

}
