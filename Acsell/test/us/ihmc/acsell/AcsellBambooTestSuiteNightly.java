package us.ihmc.acsell;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.acsell.BonoFlatGroundWalkingKinematicFootSwitchTest.class,
   us.ihmc.acsell.BonoFlatGroundWalkingTest.class
})

public class AcsellBambooTestSuiteNightly
{
   public static void main(String[] args)
   {
//      JUnitTestSuiteGenerator.generateTestSuite(AcsellBambooTestSuiteNightly.class);
   }
}

