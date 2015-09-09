package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCTurnValveBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(planType = {BambooPlanType.InDevelopment, BambooPlanType.Slow})
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
	@DeployableTestMethod(duration = 312.1)
   @Test(timeout = 1600000)
   public void testCloseValveByGrabbingCenter() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testCloseValveByGrabbingCenter();
   }
   
   @Override
	@DeployableTestMethod(duration = 97.6)
   @Test(timeout = 490000)
   public void testCloseValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testCloseValveByGrabbingRim();
   }
   
   @Override
	@DeployableTestMethod(duration = 46.6)
   @Test(timeout = 230000)
   public void testGraspValveBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testGraspValveBehavior();
   }
   
   @Override
	@DeployableTestMethod(duration = 41.6)
   @Test(timeout = 210000)
   public void testGraspValveUsingWholeBodyIKBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testGraspValveUsingWholeBodyIKBehavior();
   }
   
   @Override
	@DeployableTestMethod(duration = 53.2)
   @Test(timeout = 270000)
   public void testOpenValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testOpenValveByGrabbingRim();
   }
   
   @Override
	@DeployableTestMethod(duration = 138.6)
   @Test(timeout = 690000)
   public void testWalkToAndCloseValve() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testWalkToAndCloseValve();
   }
}
