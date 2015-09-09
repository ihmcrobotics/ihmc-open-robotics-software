package us.ihmc.graphics3DAdapter.jme.util;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.Point3f;

import org.junit.Test;

import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(planType={BambooPlanType.UI})
public class JMEPointCloudVisualizerTest
{
   public static void main(String[] args)
   {
      new JMEPointCloudVisualizerTest().testJMEPointCloudVisualizer();
   }

	@DeployableTestMethod(duration = 0.1)
	@Test(timeout = 30000)
   public void testJMEPointCloudVisualizer()
   {
      JMEPointCloudVisualizer jmePointCloudVisualizer = new JMEPointCloudVisualizer();
      
      Random random = new Random();
      
      Point3f[] randomPoint3fCloudArray = RandomTools.generateRandomPoint3fCloud(random, 10000, new Point3f(), new Point3f(5.0f, 5.0f, 5.0f));
      
      jmePointCloudVisualizer.addPointCloud(Arrays.asList(randomPoint3fCloudArray));
   }
}
