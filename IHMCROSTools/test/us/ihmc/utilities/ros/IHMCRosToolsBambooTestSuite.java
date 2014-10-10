package us.ihmc.utilities.ros;


import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
 us.ihmc.utilities.ros.ROSMessageConverterTest.class
}
)

public class IHMCRosToolsBambooTestSuite
{
public static void main(String[] args)
{
      String packageName = "us.ihmc.utilities.ros";
      System.out.println(JUnitTestSuiteConstructor.createTestSuite("IHMCRosToolsBambooTestSuite", packageName));
}
}

