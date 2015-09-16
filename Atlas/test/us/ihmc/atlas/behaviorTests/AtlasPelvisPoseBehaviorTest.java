package us.ihmc.atlas.behaviorTests;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCPelvisPoseBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.Slow, TestPlanTarget.Flaky})
public class AtlasPelvisPoseBehaviorTest extends DRCPelvisPoseBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasPelvisPoseBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 39.4)
   @Test(timeout = 200000)
   public void testPelvisPitchRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPelvisPitchRotationNoTranslation();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 33.9)
   @Test(timeout = 170000)
   public void testPelvisYawRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPelvisYawRotationNoTranslation();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 35.0)
   @Test(timeout = 170000)
   public void testPelvisRollRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPelvisRollRotationNoTranslation();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisXTranslation() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Flaky);
      super.testPelvisXTranslation();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 34.7)
   @Test(timeout = 170000)
   public void testPelvisYTranslation() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPelvisYTranslation();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 29.7)
   @Test(timeout = 150000)
   public void testSingleRandomPelvisRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testSingleRandomPelvisRotationNoTranslation();
   }
}
