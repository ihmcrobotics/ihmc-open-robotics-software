package us.ihmc.atlas;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPushInDoubleSupportTest extends DRCPushRecoveryTest
{
      private final double forceMagnitude = 900.0;
      private final double forceDuration = 0.05;
      private final Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      
      @Test
      public void testAtlasPush() throws SimulationExceededMaximumTimeException, InterruptedException
      {
         BambooTools.reportTestStartedMessage();

         String runName = "AtlasPushDuringStance";
         DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, false, false);
         
         setupAndTestPushInDoubleSupport(robotModel, runName, forceMagnitude, forceDuration, forceDirection);
      }
}
