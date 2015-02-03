package us.ihmc.acsell.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.acsell.BonoFlatGroundWalkingKinematicFootSwitchTest.class,
   us.ihmc.acsell.BonoFlatGroundWalkingTest.class
})

public class AcsellDockerTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(AcsellDockerTestSuite.class);
   }
}

