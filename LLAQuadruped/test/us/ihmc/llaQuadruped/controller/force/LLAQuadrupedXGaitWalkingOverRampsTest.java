package us.ihmc.llaQuadruped.controller.force;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkingOverRampsTest;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedXGaitWalkingOverRampsTest extends QuadrupedXGaitWalkingOverRampsTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingDownRamp() throws IOException
   {
      super.testWalkingDownRamp();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingOverShallowRamps() throws IOException
   {
      super.testWalkingOverShallowRamps();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingUpRamp() throws IOException
   {
      super.testWalkingUpRamp();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 100000)
   public void testWalkingOverAggressiveRamps() throws IOException
   {
      super.testWalkingUpRamp();
   }
}
