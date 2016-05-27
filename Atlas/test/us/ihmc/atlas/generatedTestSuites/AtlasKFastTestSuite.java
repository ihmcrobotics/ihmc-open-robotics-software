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
   us.ihmc.atlas.controllerAPI.AtlasEndToEndHeadTrajectoryMessageTest.class,
   us.ihmc.atlas.controllerAPI.AtlasEndToEndPelvisHeightTrajectoryMessageTest.class,
   us.ihmc.atlas.controllerAPI.AtlasEndToEndPelvisTrajectoryMessageTest.class,
   us.ihmc.atlas.controllerAPI.AtlasEndToEndWholeBodyTrajectoryMessageTest.class,
   us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest.class,
   us.ihmc.atlas.initialSetup.AtlasDrivingInitialSetupTest.class
})

public class AtlasKFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
