package us.ihmc.jMonkeyEngineToolkit.jme;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotics.geometry.Ray3d;

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
      cubeGraphics.translate(new Vector3D(CUBE_X, 0, -1));
      cubeGraphics.addCube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

      world.addChild(new Graphics3DNode("CubeNode", cubeGraphics));
      
      world.startWithGui();
      
      world.keepAlive(0.1); // Needs to happen so scene graph initializes properly!
      
      Ray3d ray3d = new Ray3d(new Point3D(0.0, 0.0, 0.0), new Vector3D(1.0, 0.0, 0.0));

      JMERayCollisionAdapter rayCollisionAdapter = new JMERayCollisionAdapter(world.getJMERootNode());
      rayCollisionAdapter.setPickingGeometry(ray3d);
      double pickDistance = rayCollisionAdapter.getPickDistance();
      
      assertEquals(EXPECTED_BOX_CONTACT_X, pickDistance, ERROR_TOLERANCE);
      
      world.stop();
   }
}
