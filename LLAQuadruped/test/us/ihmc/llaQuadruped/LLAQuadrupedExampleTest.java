package us.ihmc.llaQuadruped;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import us.ihmc.quadrupedRobotics.QuadrupedTestAdministrator;
import us.ihmc.quadrupedRobotics.QuadrupedTestAdministratorFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.Fast)
public class LLAQuadrupedExampleTest
{
   private static QuadrupedTestAdministratorFactory testAdministratorFactory;
   private QuadrupedTestAdministrator testAdministrator;
   
   @BeforeClass
   public static void setupClass()
   {
      try
      {
         LLAQuadrupedTestFactory llaQuadrupedTestFactory = new LLAQuadrupedTestFactory();
         testAdministratorFactory = llaQuadrupedTestFactory.createTestAdministratorFactory();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
   
   @Before
   public void setup()
   {
      try
      {
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

         testAdministratorFactory.setControlMode(QuadrupedControlMode.FORCE);
         testAdministratorFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);

         testAdministrator = testAdministratorFactory.createTestAdministrator();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
   
   @After
   public void tearDown()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @DeployableTestMethod(estimatedDuration = 10.0)
   @Test(timeout = 800000)
   public void exampleTest()
   {
      testAdministrator.simulate(1.0);
   }
}
