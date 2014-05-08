package us.ihmc.atlas;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushInSingleSupportTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPushInSingleSupportTest extends DRCPushInSingleSupportTest
{
   @Test
   public void testAtlasPush() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "AtlasPushDuringSwing";
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, false, false);
      
      setupAndTestPushInSingleSupport(robotModel, runName);
   }
}
