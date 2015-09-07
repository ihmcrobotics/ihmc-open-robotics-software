package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCTurnValveBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

@BambooPlan(planType = {BambooPlanType.InDevelopment, BambooPlanType.Slow})
public class AtlasTurnValveBehaviorTest extends DRCTurnValveBehaviorTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasTurnValveBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
      boolean useHighResolutionContactPointGrid = true;
      robotModel.createHandContactPoints(useHighResolutionContactPointGrid);
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
	@EstimatedDuration(duration = 312.1)
   @Test(timeout = 1600000)
   public void testCloseValveByGrabbingCenter() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testCloseValveByGrabbingCenter();
   }
   
   @Override
	@EstimatedDuration(duration = 97.6)
   @Test(timeout = 490000)
   public void testCloseValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testCloseValveByGrabbingRim();
   }
   
   @Override
	@EstimatedDuration(duration = 46.6)
   @Test(timeout = 230000)
   public void testGraspValveBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testGraspValveBehavior();
   }
   
   @Override
	@EstimatedDuration(duration = 41.6)
   @Test(timeout = 210000)
   public void testGraspValveUsingWholeBodyIKBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testGraspValveUsingWholeBodyIKBehavior();
   }
   
   @Override
	@EstimatedDuration(duration = 53.2)
   @Test(timeout = 270000)
   public void testOpenValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testOpenValveByGrabbingRim();
   }
   
   @Override
	@EstimatedDuration(duration = 138.6)
   @Test(timeout = 690000)
   public void testWalkToAndCloseValve() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testWalkToAndCloseValve();
   }
}
