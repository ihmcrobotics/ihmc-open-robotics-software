package us.ihmc.utilities.ros;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteConstructor;

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
      JUnitTestSuiteConstructor.generateTestSuite(IHMCRosToolsBambooTestSuite.class);
   }
}
