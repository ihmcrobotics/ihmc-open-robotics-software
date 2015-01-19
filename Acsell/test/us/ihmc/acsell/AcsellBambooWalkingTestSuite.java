package us.ihmc.acsell;

import org.junit.runner.*;
import org.junit.runners.*;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   BonoFlatGroundWalkingKinematicFootSwitchTest.class,
   BonoFlatGroundWalkingTest.class
})

public class AcsellBambooWalkingTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(AcsellBambooWalkingTestSuite.class);
   }
}
