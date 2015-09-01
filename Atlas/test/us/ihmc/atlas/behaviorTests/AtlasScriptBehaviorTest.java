package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCScriptBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;


@BambooPlan(planType = {BambooPlanType.InDevelopment, BambooPlanType.Slow})
public class AtlasScriptBehaviorTest extends DRCScriptBehaviorTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasScriptBehaviorTest()
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
   public void testPauseAndResumeScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testPauseAndResumeScript();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithOneHandPosePacket() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testScriptWithOneHandPosePacket();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithTwoComHeightScriptPackets() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testScriptWithTwoComHeightScriptPackets();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithTwoHandPosePackets() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testScriptWithTwoHandPosePackets();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testSimpleScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testSimpleScript();
   }
   
   @Override
   @EstimatedDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testStopScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testStopScript();
   }
}
