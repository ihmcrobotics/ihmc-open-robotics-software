package us.ihmc.jMonkeyEngineToolkit.jme;

import org.junit.Test;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DWorld;
import us.ihmc.robotics.geometry.shapes.Sphere3d;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Sphere;

@ContinuousIntegrationPlan(categories={IntegrationCategory.UI})
public class JMEGraphics3DWorldTest
{

	@ContinuousIntegrationTest(estimatedDuration = 1.2)
	@Test(timeout = 30000)
   public void testShowGui()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      
      world.startWithGui();
      
      world.keepAlive(1);
      world.stop();
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testWithoutGui()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      
      world.startWithoutGui();

      world.keepAlive(1);
      world.stop();
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void addASphere()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      
      world.addChild(new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3d())));
      world.startWithoutGui();

      world.keepAlive(1);
      world.stop();
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void addASphereAfterGuiStarted()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      
      world.startWithoutGui();
      world.addChild(new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3d())));

      world.keepAlive(1);
      world.stop();
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void addAJMESphere()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      
      Geometry geometry = new Geometry("jmeSphere" + "Geo", new Sphere(200, 200, 5.0f, false, true));
      Material material = new Material(((JMEGraphics3DAdapter) world.getGraphics3DAdapter()).getRenderer().getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
      material.setColor("Color", new ColorRGBA(0, 1, 0, 0.5f));
      material.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
      geometry.setMaterial(material);
      geometry.setQueueBucket(Bucket.Transparent);
      Node jmeSphereNode = new Node("jmeSphereNode");
      jmeSphereNode.move(1.5f, 2.5f, -0.5f);
      jmeSphereNode.attachChild(geometry);
      
      world.addChild(jmeSphereNode);
      world.startWithoutGui();

      world.keepAlive(1);
      world.stop();
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testSetCameraPosition()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      
      world.addChild(new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3d())));
      world.startWithGui();
      world.setCameraPosition(5, 5, 5);

      world.keepAlive(1);
      world.stop();
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void fixCameraOnSphere()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld("testWorld", new JMEGraphics3DAdapter());
      
      Graphics3DNode sphereNode = new Graphics3DNode("Sphere", new Graphics3DObject(new Sphere3d()));
      world.addChild(sphereNode);
      world.startWithGui();
      world.setCameraPosition(5, 5, 5);
      world.fixCameraOnNode(sphereNode);

      world.keepAlive(1);
      world.stop();
   }
}
