package us.ihmc.graphics3DAdapter.examples;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.NodeType;
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

//    teapotAndSphereNode.rotateAboutX(Math.PI/4.0);
      LinkGraphics teapotObject = createTeapotObject();
      LinkGraphicsInstruction sphereAppearanceHolder = teapotObject.addSphere(2.0, YoAppearance.Red());

      teapotAndSphereNode.setGraphicsObject(teapotObject);

      adapter.addRootNode(teapotAndSphereNode);


//    JPanel jPanel = adapter.getDefaultCamera().getPanel();

      NodeModifyingRunnable rotator = new NodeModifyingRunnable()
      {
         double rotation = 0.0;
         public void run()
         {
            rotation += 0.01;
            Transform3D transform = new Transform3D();
            transform.rotZ(rotation);
            node.setTransform(transform);
         }
      };
      LinkGraphicsInstructionModifyingRunnable blinker = new LinkGraphicsInstructionModifyingRunnable()
      {
         private Long previousBlink;
         private Long blinkLength = 100000L;
         boolean on = true;
         @Override
         public void run()
         {
            Long time = System.nanoTime();
            if (time - previousBlink > blinkLength)
            {
               previousBlink = time;
               toggleAppearance();
            }
         }
         private void toggleAppearance()
         {
            on = !on;
            if (on)
               instruction.setAppearance(YoAppearance.Red());
            else
               instruction.setAppearance(YoAppearance.Blue());
         }
      };

      rotator.setNode(teapotAndSphereNode);

      ArrayList<Runnable> runnables = new ArrayList<Runnable>();
      runnables.add(rotator);

      while (true)
      {
         for (Runnable runnable : runnables)
         {
            runnable.run();
         }


         Color3f color = new Color3f((float) Math.random(), (float) Math.random(), (float) Math.random());
         sphereAppearanceHolder.setAppearance(new YoAppearanceRGBColor(color));

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

   abstract class NodeModifyingRunnable implements Runnable
   {
      Graphics3DNode node;

      abstract public void run();

      public void setNode(Graphics3DNode node)
      {
         this.node = node;
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
