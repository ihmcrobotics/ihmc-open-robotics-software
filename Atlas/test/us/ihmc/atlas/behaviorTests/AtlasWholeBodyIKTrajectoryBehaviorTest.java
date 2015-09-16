package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCWholeBodyIKTrajectoryBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.InDevelopment, TestPlanTarget.Slow})
public class AtlasWholeBodyIKTrajectoryBehaviorTest extends DRCWholeBodyIKTrajectoryBehaviorTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasWholeBodyIKTrajectoryBehaviorTest()
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
   @DeployableTestMethod(estimatedDuration = 14.0)
   @Test(timeout = 70000)
   public void testConstructorAndSetInput()
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testConstructorAndSetInput();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 23.0)
   @Test(timeout = 110000)
   public void testMoveBothHandsToPose() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testMoveBothHandsToPose();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 31.6)
   @Test(timeout = 160000)
   public void testMoveOneHandToPosition() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testMoveOneHandToPosition();
   }
}
