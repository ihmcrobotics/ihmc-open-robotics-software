package us.ihmc.kalman.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.kalman.YoKalmanFilterTest.class
})

public class KalmanProjectDockerTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(KalmanProjectDockerTestSuite.class);
   }
}

