package us.ihmc.convexOptimization;


import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
   us.ihmc.convexOptimization.quadraticProgram.GenericActiveSetQPSolverTest.class
})

public class ConvexOptimizationAdapterBambooTestSuiteNightly
{
   public static void main(String[] args)
   {
      String packageName = "us.ihmc.ConvexOptimizationAdapter";
      System.out.println(JUnitTestSuiteConstructor.createTestSuite("ConvexOptimizationAdapterBambooTestSuite", packageName));
   }
}
