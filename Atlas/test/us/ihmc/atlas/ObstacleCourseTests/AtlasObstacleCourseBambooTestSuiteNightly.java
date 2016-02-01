package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
   AtlasObstacleCourseFlatTest.class,
   AtlasObstacleCourseRampsTest.class,
   AtlasObstacleCourseSteppingStonesTest.class,
   AtlasObstacleCoursePlatformTest.class,
   AtlasObstacleCourseRocksTest.class,
   AtlasObstacleCourseTrialsWalkingTaskTest.class,
   AtlasObstacleCourseTrialsTerrainTest.class,
   AtlasWallWorldTest.class
})

public class AtlasObstacleCourseBambooTestSuiteNightly
{
}
