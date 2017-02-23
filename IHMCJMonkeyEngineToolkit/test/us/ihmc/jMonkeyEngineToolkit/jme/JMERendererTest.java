package us.ihmc.jMonkeyEngineToolkit.jme;

import static org.junit.Assert.assertNotNull;

import java.util.concurrent.Callable;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DFrameListener;
import us.ihmc.jMonkeyEngineToolkit.utils.CoordinateFrameNode;
import us.ihmc.jMonkeyEngineToolkit.utils.FlatHeightMap;
import us.ihmc.robotics.geometry.shapes.Sphere3d;

@ContinuousIntegrationPlan(categories={IntegrationCategory.UI})
public class JMERendererTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testInitialization()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld(new JMEGraphics3DAdapter());
      JMERenderer renderer = world.getGraphics3DAdapter().getRenderer();
      
      world.startWithoutGui();
      assertNotNull(renderer);
      
      world.stop();
   }

   /**
    * Ground should flash on and off a few times.
    */

	@ContinuousIntegrationTest(estimatedDuration = 2.1)
	@Test(timeout = 30000)
   public void testSetGroundVisible()
   {
      JMEGraphics3DWorld world = new JMEGraphics3DWorld(new JMEGraphics3DAdapter());
      JMERenderer renderer = world.getGraphics3DAdapter().getRenderer();
      
      world.startWithGui();

      renderer.setHeightMap(new FlatHeightMap());

      for (int i = 0; i < 5; i++)
      {
         world.keepAlive(0.2);
         renderer.setGroundVisible(true);
         world.keepAlive(0.2);
         renderer.setGroundVisible(false);
      }
      
      world.stop();
   }

   /**
    * Should see a sphere orbit in a circle and leave 5 copies of itself along the way.
    */

	@ContinuousIntegrationTest(estimatedDuration = 3.1)
	@Test(timeout = 30000)
   public void testFreezeFrame()
   {
      final JMEGraphics3DWorld world = new JMEGraphics3DWorld(new JMEGraphics3DAdapter());
      JMERenderer renderer = world.getGraphics3DAdapter().getRenderer();
      
      world.startWithGui();

      final Graphics3DNode sphereNode = new Graphics3DNode("SphereNode", new Graphics3DObject(new Sphere3d(1.0), YoAppearance.Green()));
      world.addChild(sphereNode);

      world.addChild(new CoordinateFrameNode());

      world.addFrameListener(new Graphics3DFrameListener()
      {
         double totalTime = 0;

         @Override
         public void postFrame(final double timePerFrame)
         {
            world.getGraphics3DAdapter().getRenderer().enqueue(new Callable<Object>()
            {
               @Override
               public Object call() throws Exception
               {
                  double angularVelocity = 1e5;
                  double radiusOfMotion = 3;

                  totalTime += angularVelocity * timePerFrame;

                  double x = radiusOfMotion * Math.cos(totalTime);
                  double y = radiusOfMotion * Math.sin(totalTime);
                  double z = 1.0;

                  Vector3D translation = new Vector3D(x, y, z);

                  sphereNode.translateTo(translation);

                  return null;
               }
            });
         }
      });

      for (int i = 0; i < 5; i++)
      {
         world.keepAlive(0.5);

         renderer.freezeFrame(sphereNode);
      }

      world.keepAlive(0.5);
      
      world.stop();
   }
}
