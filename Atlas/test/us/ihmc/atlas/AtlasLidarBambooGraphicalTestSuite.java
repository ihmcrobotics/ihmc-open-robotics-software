package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.networkProcessor.depthData.AtlasPointCloudDataReceiverTest.class,
})

public class AtlasLidarBambooGraphicalTestSuite
{
   public static void main(String[] args)
   {
//      JUnitTestSuiteGenerator.generateTestSuite(AtlasLidarBambooGraphicalTestSuite.class);
   }
}
