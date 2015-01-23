package us.ihmc.convexOptimization;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
//  us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPSolverTest.class, 
//  us.ihmc.convexOptimization.quadraticProgram.GenericActiveSetQPSolverTest.class
})

public class ConvexOptimizationAdapterBambooTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(ConvexOptimizationAdapterBambooTestSuite.class);
   }
}
