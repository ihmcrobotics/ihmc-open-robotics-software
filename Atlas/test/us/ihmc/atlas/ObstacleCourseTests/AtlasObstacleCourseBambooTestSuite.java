package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
   AtlasObstacleCourseEveryBuildTest.class
})

public class AtlasObstacleCourseBambooTestSuite
{
   public static void main(String[] args)
   {
      String packageName = "us.ihmc.atlas.ObstacleCourseTests";
      System.out.println(JUnitTestSuiteConstructor.createTestSuite("AtlasObstacleCourseBambooTestSuite", packageName));
   }
}
