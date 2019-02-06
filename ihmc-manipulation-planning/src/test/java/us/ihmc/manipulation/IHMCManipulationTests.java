package us.ihmc.manipulation;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.manipulation.planning.gradientDescent.GradientDescentTest.class, 
   us.ihmc.manipulation.planning.manifold.ReachingManifoldVisualizingTest.class
 })

public class IHMCManipulationTests
{
   public static void main(String[] args)
   {

   }
}