package us.ihmc.convexOptimization;
import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.convexOptimization.randomSearch.RandomSearchConvexOptimizationAdapterTest.class
//  us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPSolverTest.class, 
//  us.ihmc.convexOptimization.quadraticProgram.GenericActiveSetQPSolverTest.class
})

public class ConvexOptimizationAdapterBambooTestSuite
{
   public static void main(String[] args)
   {
//      JUnitTestSuiteGenerator.generateTestSuite(ConvexOptimizationAdapterBambooTestSuite.class);
   }
}
