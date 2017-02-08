package us.ihmc.robotics.math.corruptors;

import java.util.Random;

import javax.vecmath.Matrix3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.tools.testing.JUnitTools;

public class NoisyYoRotationMatrixTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout=300000)
   public void testNoNoise()
   {
      Random random = new Random(176L);
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      NoisyYoRotationMatrix mat = new NoisyYoRotationMatrix("rot", registry);
      mat.setBiasRandomlyBetweenMinAndMax();
      mat.setRandomBound(1.0);
      mat.setNoiseType(NoiseType.GAUSSIAN);
      mat.setGaussianNoise(1.0);
      mat.setIsNoisy(false);

      Matrix3d in = new Matrix3d();
      double yaw = random.nextDouble();
      double pitch = random.nextDouble();
      double roll = random.nextDouble();
      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, in);
      
      mat.update(in);
      Matrix3d out = mat.getMatrix3d();
      JUnitTools.assertMatrix3dEquals("", in, out, 0.0);
   }

}
