package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
	ValkyrieFlatGroundWalkingTest.class,
})

public class ValkyrieSimpleBambooValkyrieWalkingTestSuite
{
	public static void main(String[] args)
	{
		JUnitTestSuiteConstructor.generateTestSuite(ValkyrieSimpleBambooValkyrieWalkingTestSuite.class);
	}
}
