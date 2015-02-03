package us.ihmc.ihmcPerception.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.ihmcPerception.footstepPlanners.ADStarFootstepPlannerTest.class,
   us.ihmc.ihmcPerception.footstepPlanners.AStarFootstepPlannerTest.class,
   us.ihmc.ihmcPerception.footstepPlanners.DStarFootstepPlannerTest.class,
   us.ihmc.ihmcPerception.footstepPlanners.FootstepGeneratorsTest.class,
   us.ihmc.ihmcPerception.footstepPlanners.SemiCircularStepValidityMetricTest.class,
   us.ihmc.ihmcPerception.footstepSnapper.ConvexHullFootstepSnapperTest.class,
   us.ihmc.ihmcPerception.footstepSnapper.FootstepSnapperTest.class,
   us.ihmc.ihmcPerception.footstepSnapper.HeightMapBestFitPlaneCalculatorTest.class,
   us.ihmc.ihmcPerception.footstepSnapper.QuickHull3DTest.class
})

public class IHMCPerceptionDockerTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(IHMCPerceptionDockerTestSuite.class);
   }
}

