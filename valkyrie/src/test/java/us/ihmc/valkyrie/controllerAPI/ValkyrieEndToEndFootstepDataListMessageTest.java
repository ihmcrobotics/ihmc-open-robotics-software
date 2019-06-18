package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.QueueableMessage;
import us.ihmc.avatar.controllerAPI.EndToEndFootstepDataListMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndFootstepDataListMessageTest extends EndToEndFootstepDataListMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Test
   public void testProblematicNasaMessage() throws SimulationExceededMaximumTimeException
   {
      testMessageIsHandled(createProblematicNasaMessage());
   }

   private FootstepDataListMessage createProblematicNasaMessage()
   {
      FootstepDataListMessage message = new FootstepDataListMessage();

      FootstepDataMessage step1 = message.getFootstepDataList().add();
      step1.setRobotSide((byte) 0);
      step1.getLocation().set(-0.0453, 0.169, 0.0);
      step1.getOrientation().set(0.0, 0.0, 0.13052, 0.9914486);
      step1.getPredictedContactPoints2d().clear();
      step1.setTrajectoryType((byte) 0);
      step1.setSwingHeight(0.1);
      step1.getCustomPositionWaypoints().clear();
      step1.getSwingTrajectory().clear();
      step1.setSwingTrajectoryBlendDuration(0.0);
      step1.setSwingDuration(0.0);
      step1.setTransferDuration(0.0);
      step1.setTouchdownDuration(-1.0);
      step1.setSequenceId(1L);

      FootstepDataMessage step2 = message.getFootstepDataList().add();
      step2.setRobotSide((byte) 1);
      step2.getLocation().set(0.0453, -0.169, 0.0);
      step2.getOrientation().set(0.0, 0.0, 0.13052, 0.9914486);
      step2.getPredictedContactPoints2d().clear();
      step2.setTrajectoryType((byte) 0);
      step2.setSwingHeight(0.1);
      step2.getCustomPositionWaypoints().clear();
      step2.getSwingTrajectory().clear();
      step2.setSwingTrajectoryBlendDuration(0.0);
      step2.setSwingDuration(0.0);
      step2.setTransferDuration(0.0);
      step2.setTouchdownDuration(-1.0);
      step2.setSequenceId(1L);

      FootstepDataMessage step3 = message.getFootstepDataList().add();
      step3.setRobotSide((byte) 0);
      step3.getLocation().set(-0.0875, 0.1516, 0.0);
      step3.getOrientation().set(0.0, 0.0, 0.2588, 0.9659);
      step3.getPredictedContactPoints2d().clear();
      step3.setTrajectoryType((byte) 0);
      step3.setSwingHeight(0.1);
      step3.getCustomPositionWaypoints().clear();
      step3.getSwingTrajectory().clear();
      step3.setSwingTrajectoryBlendDuration(0.0);
      step3.setSwingDuration(0.0);
      step3.setTransferDuration(0.0);
      step3.setTouchdownDuration(-1.0);
      step3.setSequenceId(1L);

      message.setExecutionTiming((byte) 0);
      message.setDefaultSwingDuration(1.20000004768);
      message.setDefaultTransferDuration(1.0);
      message.setFinalTransferDuration(1.0);
      message.setSequenceId(1L);

      message.setTrustHeightOfFootsteps(false);
      message.setAreFootstepsAdjustable(false);
      message.setOffsetFootstepsWithExecutionError(false);
      QueueableMessage queueingProperties = message.getQueueingProperties();
      queueingProperties.setSequenceId(1);
      queueingProperties.setExecutionMode((byte) 0);
      queueingProperties.setMessageId(1L);
      queueingProperties.setPreviousMessageId(0);
      queueingProperties.setExecutionDelayTime(0.0);

      return message;
   }
}
