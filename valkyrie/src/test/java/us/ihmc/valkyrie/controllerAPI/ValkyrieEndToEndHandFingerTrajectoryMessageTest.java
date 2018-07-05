package us.ihmc.valkyrie.controllerAPI;

import org.junit.Test;

import controller_msgs.msg.dds.HandFingerTrajectoryMessage;
import us.ihmc.avatar.controllerAPI.EndToEndHandFingerTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.fingers.ValkyrieFingerControlParameters;
import us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName;

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
   public void testMessageConverter() throws SimulationExceededMaximumTimeException
   {
      super.testMessageConverter();
   }

   @Override
   public HandFingerTrajectoryMessage createHandFingerTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration)
   {
      ValkyrieFingerMotorName[] valkyrieFingerMotorNames = ValkyrieFingerMotorName.values;
      int numberOfFingerMotors = valkyrieFingerMotorNames.length;

      double trajectoryTime = 3.0;
      double delayTime = 2.0;

      double[] trajectoryTimes = new double[numberOfFingerMotors];
      double[] desiredJointPositions = new double[numberOfFingerMotors];
      double[] executionDelayTimes = new double[numberOfFingerMotors];

      for (int i = 0; i < numberOfFingerMotors; i++)
      {
         trajectoryTimes[i] = trajectoryTime;

         switch (handConfiguration)
         {
         case CLOSE:
            desiredJointPositions[i] = ValkyrieFingerControlParameters.getClosedDesiredDefinition(robotSide).get(valkyrieFingerMotorNames[i]);
            if (valkyrieFingerMotorNames[i].getFingerName() == FingerName.THUMB)
               executionDelayTimes[i] = delayTime;
            break;
         case OPEN:
            desiredJointPositions[i] = ValkyrieFingerControlParameters.getOpenDesiredDefinition(robotSide).get(valkyrieFingerMotorNames[i]);
            if (valkyrieFingerMotorNames[i].getFingerName() != FingerName.THUMB)
               executionDelayTimes[i] = delayTime;
            break;
         default:
            break;
         }
      }

      return HumanoidMessageTools.createHandFingerTrajectoryMessage(robotSide, trajectoryTimes, desiredJointPositions, executionDelayTimes);
   }
}
