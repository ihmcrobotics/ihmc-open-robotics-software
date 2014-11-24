package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
	ValkyrieFlatGroundWalkingTest.class,
})

public class ValkyrieBambooWalkingTestSuite
{
	public static void main(String[] args)
	{
		String packageName = "us.ihmc.valkyrie";
		System.out.println(JUnitTestSuiteConstructor.createTestSuite("ValkyrieBambooWalkingTestSuite", packageName));
	}
}
