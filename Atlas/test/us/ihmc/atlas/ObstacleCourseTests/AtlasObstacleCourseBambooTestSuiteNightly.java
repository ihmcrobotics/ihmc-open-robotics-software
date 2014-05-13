package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
   AtlasObstacleCoursePlatformTest.class,
   AtlasObstacleCourseRocksTest.class,
   AtlasObstacleCourseTrialsWalkingTaskTest.class,
   AtlasObstacleCourseTrialsTerrainTest.class
})

public class AtlasObstacleCourseBambooTestSuiteNightly
{
}
