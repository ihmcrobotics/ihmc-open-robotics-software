package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class JMEDataTypeUtilsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransforms()
   {
      for (int i = 0; i < 1000; i++)
      {
         RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(new Random(-2346283641976L));
         //TODO @Davide test removed
         //Transform jmeVersion = JMEDataTypeUtils.j3dTransform3DToJMETransform(randomTransform);
         //RigidBodyTransform resultTransform = JMEDataTypeUtils.jmeTransformToTransform3D(jmeVersion);
         //JUnitTools.assertTransformEquals(randomTransform, resultTransform, 1e-6);
      }
   }

}
