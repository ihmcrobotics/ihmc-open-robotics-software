package us.ihmc.darpaRoboticsChallenge.ros;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ROSMessageGenerationTest
{
	@DeployableTestMethod(duration = 0.1)
   @Test(timeout = 30000)
   public void testROSMEssageGenerator() throws Exception
   {
      DRCROSMessageGenerator.generate();
   }
}
