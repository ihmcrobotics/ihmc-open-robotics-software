package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushInSingleSupportTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ValkyriePushInSingleSupportTest extends DRCPushInSingleSupportTest
{
   @Test
   public void testValkyriePush() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "ValkyriePushDuringSwing";
      DRCRobotModel robotModel = new ValkyrieRobotModel(false);
      
      setupAndTestPushInSingleSupport(robotModel, runName);
   }
}