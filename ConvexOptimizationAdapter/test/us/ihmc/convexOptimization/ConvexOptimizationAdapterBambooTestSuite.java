package us.ihmc.convexOptimization;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   
})

public class ConvexOptimizationAdapterBambooTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(ConvexOptimizationAdapterBambooTestSuite.class);
   }
}
