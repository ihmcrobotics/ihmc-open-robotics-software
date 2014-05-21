package us.ihmc.valkyrie;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCPushInSingleSupportTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ValkyriePushInSingleSupportTest extends DRCPushInSingleSupportTest
{
   private final double forceMagnitude = 1000.0;
   private final double forceDuration = 0.05;
   private final Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
   
   @Test
   public void testValkyriePush() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "ValkyriePushDuringSwing";
      DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
      
      setupAndTestPushInSingleSupport(robotModel, runName, forceMagnitude, forceDuration, forceDirection);
   }
}