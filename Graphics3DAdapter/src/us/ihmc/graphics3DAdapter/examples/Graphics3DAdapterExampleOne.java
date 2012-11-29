package us.ihmc.graphics3DAdapter.examples;

import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.graphics.LinkGraphics;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.graphics3DAdapter.graphics.instructions.LinkGraphicsInstruction;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public class Graphics3DAdapterExampleOne
{

   
 
   public void doExampleOne(Graphics3DAdapter adapter)
   {
      Graphics3DNode teapotAndSphereNode = new Graphics3DNode("teaPot");
       
//      teapotAndSphereNode.rotateAboutX(Math.PI/4.0);
     
      LinkGraphics teapotObject = new LinkGraphics(); 
      teapotObject.translate(0.0, 1.0, 1.0);
      teapotObject.rotate(Math.PI/4.0, LinkGraphics.X);
      teapotObject.addEllipsoid(2.0, 2.0, 1.5);
      teapotObject.identity(); 
      teapotObject.translate(0.0, 2.0, 1.0);
      teapotObject.rotate(Math.PI/4.0, LinkGraphics.X);
      LinkGraphicsInstruction sphereAppearanceHolder = teapotObject.addSphere(2.0, YoAppearance.Red());
      
      teapotAndSphereNode.setGraphicsObject(teapotObject);
      
      adapter.addRootNode(teapotAndSphereNode);
      
     
//      JPanel jPanel = adapter.getDefaultCamera().getPanel();
   
   
      
      double rotation = 0.0;
      while(true)
      {
         rotation += 0.01;
         Transform3D transform = new Transform3D();
         
         transform.rotZ(rotation);
         
         teapotAndSphereNode.setTransform(transform);
         
         Color3f color = new Color3f((float)Math.random(), (float)Math.random(), (float)Math.random());
         sphereAppearanceHolder.setAppearance(new YoAppearanceRGBColor(color));
         try
         {
            Thread.sleep(10L);
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }
   }
}
