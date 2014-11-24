package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
	AtlasFlatGroundWalkingTest.class,
})

public class AtlasBambooWalkingTestSuite
{
	public static void main(String[] args)
	{
		String packageName = "us.ihmc.atlas";
		System.out.println(JUnitTestSuiteConstructor.createTestSuite("AtlasBambooWalkingTestSuite", packageName));
	}
}