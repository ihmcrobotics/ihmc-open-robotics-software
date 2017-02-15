package us.ihmc.graphicsDescription;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class Graphics3DObjectTest
{
   private static final double CUBE_SIDE = 2.0;
   private static final double CUBE_X = 5.0;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testValidCubeGraphics()
   {
      Graphics3DObject cubeGraphics = new Graphics3DObject();
      cubeGraphics.translate(new Vector3d(CUBE_X, 0.0, 0.0));
      cubeGraphics.addCube(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
      
      assertEquals(2, cubeGraphics.getGraphics3DInstructions().size());
   }
}
