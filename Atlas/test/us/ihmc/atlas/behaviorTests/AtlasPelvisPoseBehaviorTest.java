package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCPelvisPoseBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.Flaky})
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
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisPitchRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testPelvisPitchRotationNoTranslation();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisYawRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testPelvisYawRotationNoTranslation();
   }

   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisRollRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testPelvisRollRotationNoTranslation();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisXTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Flaky);
      super.testPelvisXTranslation();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testPelvisYTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testPelvisYTranslation();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testSingleRandomPelvisRotationNoTranslation() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testSingleRandomPelvisRotationNoTranslation();
   }
}
