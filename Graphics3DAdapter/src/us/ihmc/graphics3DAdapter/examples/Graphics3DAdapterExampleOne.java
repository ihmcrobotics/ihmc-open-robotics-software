package us.ihmc.graphics3DAdapter.examples;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.graphics.GraphicsObject;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

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
      AppearanceHolder sphereAppearanceHolder = teapotObject.addSphere(radius);
      
      teapotAndSphereNode.setGraphicsObject(sphereObject);
      
      adapter.addRootGraphicsNode(teaPotAndSphereNode);
      
     
      JPanel jPanel = adapter.getDefaultCamera().getPanel();
   
   
      double rotation = 0.0;
      while(true)
      {
         rotation += 0.001;
         Transform transform = new Transform();
         
         transform.rotateAboutX(rotation)
         
         teapotAndSphereNode.setTransform(transform);
         
         Color3f color = new Color3f(Math.random(), Math.random(), Math.random());
         sphereAppearanceHolder.setAppearance(new ColorAppearance(color));
         sleep(10L);
      }
   }
}
