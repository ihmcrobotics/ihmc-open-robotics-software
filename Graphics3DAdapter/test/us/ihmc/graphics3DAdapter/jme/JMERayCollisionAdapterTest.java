package us.ihmc.graphics3DAdapter.jme;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotics.geometry.Ray3d;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories={IntegrationCategory.UI})
public class JMERayCollisionAdapterTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testObjectPicking()
   {
//      ThreadTools.sleep(10000); // Put this in to give me time to attach the debugger to this test.
      JMEGraphics3DWorld world = new JMEGraphics3DWorld(new JMEGraphics3DAdapter());
      
      double CUBE_SIDE = 2.0;
      double CUBE_X = 2.0;
      double ERROR_TOLERANCE = 1e-6;
      double EXPECTED_BOX_CONTACT_X = 1.0;
      
      Graphics3DObject cubeGraphics = new Graphics3DObject();
      cubeGraphics.translate(new Vector3d(CUBE_X, 0, -1));
      cubeGraphics.addCube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

      world.addChild(new Graphics3DNode("CubeNode", cubeGraphics));
      
      world.startWithGui();
      
      world.keepAlive(0.1); // Needs to happen so scene graph initializes properly!
      
      Ray3d ray3d = new Ray3d(new Point3d(0.0, 0.0, 0.0), new Vector3d(1.0, 0.0, 0.0));

      JMERayCollisionAdapter rayCollisionAdapter = new JMERayCollisionAdapter(world.getJMERootNode());
      rayCollisionAdapter.setPickingGeometry(ray3d);
      double pickDistance = rayCollisionAdapter.getPickDistance();
      
      assertEquals(EXPECTED_BOX_CONTACT_X, pickDistance, ERROR_TOLERANCE);
      
      world.stop();
   }
}
