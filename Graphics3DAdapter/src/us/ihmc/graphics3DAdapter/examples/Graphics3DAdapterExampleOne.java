package us.ihmc.graphics3DAdapter.examples;

import javax.media.j3d.Transform3D;
import javax.swing.JPanel;
import javax.vecmath.Color3f;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.graphics.GraphicsObject;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

import com.sun.org.apache.xalan.internal.xsltc.cmdline.Transform;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.graphics.YoAppearanceRGBColor;
import com.yobotics.simulationconstructionset.robotdefinition.linkgraphicinstructions.LinkGraphicsInstruction;

public class Graphics3DAdapterExampleOne
{

   
 
   public void doExampleOne(Graphics3DAdapter adapter)
   {
      Graphics3DNode teapotAndSphereNode = new Graphics3DNode();
       
//      teapotAndSphereNode.rotateAboutX(Math.PI/4.0);
     
      GraphicsObject teapotObject = new GraphicsObject(); 
      teapotObject.translate(0.0, 1.0, 1.0);
      teapotObject.rotate(Math.PI/4.0, GraphicsObject.X);
      teapotObject.addEllipsoid(2.0, 2.0, 1.5);
      teapotObject.identity(); 
      teapotObject.translate(0.0, 2.0, 1.0);
      teapotObject.rotate(Math.PI/4.0, GraphicsObject.X);
      LinkGraphicsInstruction sphereAppearanceHolder = teapotObject.addSphere(2.0, YoAppearance.Red());
      
      teapotAndSphereNode.setGraphicsObject(teapotObject);
      
      adapter.addRootNode(teapotAndSphereNode);
      
     
      JPanel jPanel = adapter.getDefaultCamera().getPanel();
   
   
      double rotation = 0.0;
      while(true)
      {
         rotation += 0.001;
         Transform3D transform = new Transform3D();
         
         transform.rotX(rotation);
         
         teapotAndSphereNode.setTransform(transform);
         
         Color3f color = new Color3f((float)Math.random(), (float)Math.random(), (float)Math.random());
         sphereAppearanceHolder.setAppearance(new YoAppearanceRGBColor(color));
         try
         {
            Thread.sleep(100L);
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }
   }
}
