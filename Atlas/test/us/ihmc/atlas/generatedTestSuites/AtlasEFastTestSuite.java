package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses
({
   us.ihmc.atlas.hikSim.AtlasWholeBodyIkSolverTest.class,
   us.ihmc.atlas.hikSim.AtlasWholeBodyTrajectoryTest.class,
   us.ihmc.atlas.initialSetup.AtlasDrivingInitialSetupTest.class,
   us.ihmc.atlas.momentumBasedControl.AtlasOptimizationMomentumControlModuleTest.class
})

public class AtlasEFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
