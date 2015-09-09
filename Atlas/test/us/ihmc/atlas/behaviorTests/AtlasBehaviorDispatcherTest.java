package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.BehaviorDispatcherTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(planType = {BambooPlanType.InDevelopment, BambooPlanType.Slow})
public class AtlasBehaviorDispatcherTest extends BehaviorDispatcherTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasBehaviorDispatcherTest()
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
	@DeployableTestMethod(duration = 50.0)
   @Test(timeout = 250000)
   public void testDispatchPelvisPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testDispatchPelvisPoseBehavior();
   }
   
   @Override
	@DeployableTestMethod(duration = 315.0)
   @Test(timeout = 1600000)
   public void testDispatchWalkToLocationBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testDispatchWalkToLocationBehavior();
   }
   
   @Override
	@DeployableTestMethod(duration = 33.4)
   @Test(timeout = 170000)
   public void testDispatchWalkToLocationBehaviorAndStop() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testDispatchWalkToLocationBehaviorAndStop();
   }
   
   @Override
	@DeployableTestMethod(duration = 314.6)
   @Test(timeout = 1600000)
   public void testDispatchWalkToLocationBehaviorPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testDispatchWalkToLocationBehaviorPauseAndResume();
   }
}
