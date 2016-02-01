package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCTurnValveBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = {TestPlanTarget.Slow})
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
   @DeployableTestMethod(estimatedDuration = 312.1, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 1600000)
   public void testCloseValveByGrabbingCenter() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      super.testCloseValveByGrabbingCenter();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 97.6, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 490000)
   public void testCloseValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      super.testCloseValveByGrabbingRim();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 50.4)
   @Test(timeout = 250000)
   public void testGraspValveBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      super.testGraspValveBehavior();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 41.6, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 210000)
   public void testGraspValveUsingWholeBodyIKBehavior() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      super.testGraspValveUsingWholeBodyIKBehavior();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 53.2, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 270000)
   public void testOpenValveByGrabbingRim() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      super.testOpenValveByGrabbingRim();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 138.6, targets = TestPlanTarget.InDevelopment)
   @Test(timeout = 690000)
   public void testWalkToAndCloseValve() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      super.testWalkToAndCloseValve();
   }
}
