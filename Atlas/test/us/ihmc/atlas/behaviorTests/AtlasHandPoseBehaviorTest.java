package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCHandPoseBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;


@BambooPlan(planType = {BambooPlanType.InDevelopment, BambooPlanType.Slow})
public class AtlasHandPoseBehaviorTest extends DRCHandPoseBehaviorTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasHandPoseBehaviorTest()
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
	@EstimatedDuration(duration = 26.2)
   @Test(timeout = 130000)
   public void testAimPalmNormalXaxis() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testAimPalmNormalXaxis();
   }
   
   @Override
	@EstimatedDuration(duration = 28.1)
   @Test(timeout = 140000)
   public void testAimPalmNormalXaxisZupGrasp() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testAimPalmNormalXaxisZupGrasp();
   }
   
   @Override
	@EstimatedDuration(duration = 26.1)
   @Test(timeout = 130000)
   public void testAimPalmNormalYaxis() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testAimPalmNormalYaxis();
   }
   
   @Override
	@EstimatedDuration(duration = 26.6)
   @Test(timeout = 130000)
   public void testAimPalmNormalYaxisZupGrasp() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testAimPalmNormalYaxisZupGrasp();
   }
   
   @Override
	@EstimatedDuration(duration = 33.2)
   @Test(timeout = 170000)
   public void testJointSpaceHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testJointSpaceHandPoseMove();
   }
   
   @Override
	@EstimatedDuration(duration = 42.2)
   @Test(timeout = 210000)
   public void testMoveHandToHome() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testMoveHandToHome();
   }
   
   @Override
	@EstimatedDuration(duration = 43.4)
   @Test(timeout = 220000)
   public void testPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testPauseAndResume();
   }
   
   @Override
	@EstimatedDuration(duration = 34.1)
   @Test(timeout = 170000)
   public void testStop() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testStop();
   }
   
   @Override
	@EstimatedDuration(duration = 57.8)
   @Test(timeout = 290000)
   public void testTaskSpaceMoveToPoseAchievedInJointSpace() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testTaskSpaceMoveToPoseAchievedInJointSpace();
   }
   
   @Override
	@EstimatedDuration(duration = 35.2)
   @Test(timeout = 180000)
   public void testTwoSequentialHandPoses() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testTwoSequentialHandPoses();
   }
   
   @Override
	@EstimatedDuration(duration = 35.9)
   @Test(timeout = 180000)
   public void testTwoSimultaneousHandPoseBehaviors() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testTwoSimultaneousHandPoseBehaviors();
   }
   
   @Override
	@EstimatedDuration(duration = 27.6)
   @Test(timeout = 140000)
   public void testUnreachableHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testUnreachableHandPoseMove();
   }
}
