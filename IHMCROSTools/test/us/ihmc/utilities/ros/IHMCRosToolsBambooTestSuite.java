package us.ihmc.utilities.ros;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteGenerator;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.utilities.ros.ROSMessageConverterTest.class,
//   us.ihmc.utilities.ros.RosServiceClientTest.class
})

public class IHMCRosToolsBambooTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteGenerator.generateTestSuite(IHMCRosToolsBambooTestSuite.class);
   }
}
