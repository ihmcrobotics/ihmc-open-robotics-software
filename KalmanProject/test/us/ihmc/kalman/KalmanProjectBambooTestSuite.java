package us.ihmc.kalman;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.kalman.YoKalmanFilterTest.class
})

public class KalmanProjectBambooTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(KalmanProjectBambooTestSuite.class);
   }
}
