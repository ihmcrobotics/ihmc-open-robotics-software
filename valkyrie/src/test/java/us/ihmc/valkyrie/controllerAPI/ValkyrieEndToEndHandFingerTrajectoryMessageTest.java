package us.ihmc.valkyrie.controllerAPI;

import org.junit.Test;

import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.avatar.controllerAPI.EndToEndHandFingerTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.packets.Packet;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.fingers.ValkyrieHandFingerTrajectoryMessageConversion;

public class ValkyrieEndToEndHandFingerTrajectoryMessageTest extends EndToEndHandFingerTrajectoryMessageTest
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

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testCloseAndStopAndOpen() throws SimulationExceededMaximumTimeException
   {
      super.testCloseAndStopAndOpen();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testBasicGrip() throws SimulationExceededMaximumTimeException
   {
      super.testBasicGrip();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testCloseAndOpenFingers() throws SimulationExceededMaximumTimeException
   {
      super.testCloseAndOpenFingers();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 45.9)
   @Test(timeout = 230000)
   public void testClose() throws SimulationExceededMaximumTimeException
   {
      super.testClose();
   }

   @Override
   public Packet<?> createTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration)
   {
      ValkyrieHandFingerTrajectoryMessage message = new ValkyrieHandFingerTrajectoryMessage();

      ValkyrieHandFingerTrajectoryMessageConversion.convertHandConfiguration(robotSide, handConfiguration, message);

      return message;
   }
}
