package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

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
//      System.out.println(JUnitTestSuiteGenerator.createTestSuite("AtlasObstacleCourseBambooTestSuite", packageName, "test/"));
   }
}
