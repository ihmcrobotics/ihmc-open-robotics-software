package us.ihmc.robotics.robotDescription;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class RobotDescriptionTest
{

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOne()
   {
      RobotDescription description = new RobotDescription("Test");
      assertEquals("Test", description.getName());

      description.setName("TestTwo");
      assertEquals("TestTwo", description.getName());


   }


}
