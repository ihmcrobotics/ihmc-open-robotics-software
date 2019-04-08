package us.ihmc.valkyrie.controllerAPI;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.avatar.controllerAPI.EndToEndHandFingerTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.fingers.ValkyrieFingerControlParameters;
import us.ihmc.valkyrie.fingers.ValkyrieFingerMotorName;
import us.ihmc.valkyrie.fingers.ValkyrieHandFingerTrajectoryMessageConversion;
import us.ihmc.valkyrie.fingers.ValkyrieHandJointName;

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
   @Test
   public void testCloseAndStopAndOpen() throws SimulationExceededMaximumTimeException
   {
      super.testCloseAndStopAndOpen();
   }

   @Override
   @Test
   public void testCloseAndOpenFingers() throws SimulationExceededMaximumTimeException
   {
      super.testCloseAndOpenFingers();
   }

   @Override
   @Test
   public void testClose() throws SimulationExceededMaximumTimeException
   {
      super.testClose();
   }

   @Override
   public ValkyrieHandFingerTrajectoryMessage createTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration)
   {
      ValkyrieHandFingerTrajectoryMessage message = new ValkyrieHandFingerTrajectoryMessage();

      ValkyrieHandFingerTrajectoryMessageConversion.convertHandConfiguration(robotSide, handConfiguration, message);

      return message;
   }

   @Override
   public void assertDesiredFingerJoint(RobotSide robotSide, HandConfiguration handConfiguration, double epsilon)
   {
      ValkyrieHandFingerTrajectoryMessage valkyrieFingerTrajectoryMessage = createTrajectoryMessage(robotSide, handConfiguration);

      int numberOfFingers = valkyrieFingerTrajectoryMessage.getValkyrieFingerMotorNames().size();

      for (int i = 0; i < numberOfFingers; i++)
      {
         double desiredPosition = getExpectedFingerMotorPosition(valkyrieFingerTrajectoryMessage, i);

         double[] expectedHandJointPositions = getExpectedHandJointPosition(valkyrieFingerTrajectoryMessage, i, desiredPosition);
         double[] handJointPositions = getCurrentHandJointPosition(valkyrieFingerTrajectoryMessage, i);

         for (int j = 0; j < handJointPositions.length; j++)
         {
            assertEquals(expectedHandJointPositions[j], handJointPositions[j], epsilon);
         }
      }
   }

   private double getExpectedFingerMotorPosition(ValkyrieHandFingerTrajectoryMessage valkyrieFingerTrajectoryMessage, int indexOfFinger)
   {
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = valkyrieFingerTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
      Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfFinger).getTrajectoryPoints();
      TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPoints.getLast();
      double desiredPosition = trajectoryPoint1DMessage.getPosition();

      return desiredPosition;
   }

   private double[] getExpectedHandJointPosition(ValkyrieHandFingerTrajectoryMessage valkyrieFingerTrajectoryMessage, int indexOfFinger, double desiredPosition)
   {
      ValkyrieFingerMotorName fingerMotorName = ValkyrieFingerMotorName.fromByte(valkyrieFingerTrajectoryMessage.getValkyrieFingerMotorNames().get(indexOfFinger));
      RobotSide robotSide = RobotSide.fromByte(valkyrieFingerTrajectoryMessage.getRobotSide());

      ArrayList<OneDegreeOfFreedomJoint> handJoints = getAllHandJoints(robotSide, fingerMotorName);

      double[] handJointPositions = new double[handJoints.size()];
      for (int i = 0; i < handJoints.size(); i++)
      {
         ValkyrieHandJointName handJointName = fingerMotorName.getCorrespondingJointName(i);
         handJointPositions[i] = ValkyrieFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, desiredPosition);
      }

      return handJointPositions;
   }

   private double[] getCurrentHandJointPosition(ValkyrieHandFingerTrajectoryMessage valkyrieFingerTrajectoryMessage, int indexOfFinger)
   {
      ValkyrieFingerMotorName fingerMotorName = ValkyrieFingerMotorName.fromByte(valkyrieFingerTrajectoryMessage.getValkyrieFingerMotorNames().get(indexOfFinger));
      RobotSide robotSide = RobotSide.fromByte(valkyrieFingerTrajectoryMessage.getRobotSide());

      ArrayList<OneDegreeOfFreedomJoint> handJoints = getAllHandJoints(robotSide, fingerMotorName);

      double[] handJointPositions = new double[handJoints.size()];
      for (int i = 0; i < handJoints.size(); i++)
      {
         handJointPositions[i] = handJoints.get(i).getQ();
      }

      return handJointPositions;
   }

   private ArrayList<OneDegreeOfFreedomJoint> getAllHandJoints(RobotSide robotSide, ValkyrieFingerMotorName fingerMotorName)
   {
      ArrayList<OneDegreeOfFreedomJoint> handJoints = new ArrayList<OneDegreeOfFreedomJoint>();

      ValkyrieHandJointName firstHandJointName = fingerMotorName.getCorrespondingJointName(0);
      Joint firstHandJoint = controllerFullRobotModel.getJoint(firstHandJointName.getJointName(robotSide));

      if (fingerMotorName == ValkyrieFingerMotorName.ThumbMotorRoll)
         handJoints.add((OneDegreeOfFreedomJoint) firstHandJoint);
      else
         firstHandJoint.recursiveGetOneDegreeOfFreedomJoints(handJoints);

      return handJoints;
   }
}
