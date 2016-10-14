package us.ihmc.convexOptimization.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.convexOptimization.quadraticProgram.ConstrainedQPSolverTest.class,
   us.ihmc.convexOptimization.quadraticProgram.GenericActiveSetQPSolverTest.class,
   us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPSolverTest.class,
   us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolverTest.class,
   us.ihmc.convexOptimization.quadraticProgram.SimpleInefficientEqualityConstrainedQPSolverTest.class,
   us.ihmc.convexOptimization.randomSearch.RandomSearchConvexOptimizationAdapterTest.class
})

public class ConvexOptimizationAdapterAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
