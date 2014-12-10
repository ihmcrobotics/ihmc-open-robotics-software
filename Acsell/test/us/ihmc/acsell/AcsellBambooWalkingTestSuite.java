package us.ihmc.acsell;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
	BonoFlatGroundWalkingTest.class,
})

public class AcsellBambooWalkingTestSuite
{
	public static void main(String[] args)
	{
      JUnitTestSuiteConstructor.generateTestSuite(AcsellBambooWalkingTestSuite.class);
	}
}
