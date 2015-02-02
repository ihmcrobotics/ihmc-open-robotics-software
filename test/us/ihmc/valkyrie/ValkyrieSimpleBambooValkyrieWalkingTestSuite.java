package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.generator.JUnitTestSuiteGenerator;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
	ValkyrieFlatGroundWalkingTest.class,
})

public class ValkyrieSimpleBambooValkyrieWalkingTestSuite
{
	public static void main(String[] args)
	{
		JUnitTestSuiteGenerator.generateTestSuite(ValkyrieSimpleBambooValkyrieWalkingTestSuite.class);
	}
}
