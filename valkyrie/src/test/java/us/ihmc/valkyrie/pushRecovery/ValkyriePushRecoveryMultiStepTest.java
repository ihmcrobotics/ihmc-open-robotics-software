package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCPushRecoveryMultiStepTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Disabled
public class ValkyriePushRecoveryMultiStepTest extends DRCPushRecoveryMultiStepTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   protected void setForwardPushParameters()
   {
      forceMagnitude = 350.0;
      forceDuration = 0.2;
   }

   @Override
   protected void setBackwardPushParameters()
   {
      forceMagnitude = -400.0;
      forceDuration = 0.2;
   }

   @Override
   @Test
   public void testMultiStepBackwardAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testMultiStepBackwardAndContinueWalking();
   }

   @Override
   @Test
   @Disabled("Needs to be improved")
   public void testMultiStepForwardAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testMultiStepForwardAndContinueWalking();
   }
}
