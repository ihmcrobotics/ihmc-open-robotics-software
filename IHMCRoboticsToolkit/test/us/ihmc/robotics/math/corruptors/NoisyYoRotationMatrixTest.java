package us.ihmc.robotics.math.corruptors;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

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

      RotationMatrix in = new RotationMatrix();
      double yaw = random.nextDouble();
      double pitch = random.nextDouble();
      double roll = random.nextDouble();
      in.setYawPitchRoll(yaw, pitch, roll);
      
      mat.update(in);
      RotationMatrix out = mat.getMatrix3d();
      EuclidCoreTestTools.assertMatrix3DEquals("", in, out, 0.0);
   }

}
