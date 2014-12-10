package us.ihmc.utilities.ros;

import org.junit.runner.*;
import org.junit.runners.*;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

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
