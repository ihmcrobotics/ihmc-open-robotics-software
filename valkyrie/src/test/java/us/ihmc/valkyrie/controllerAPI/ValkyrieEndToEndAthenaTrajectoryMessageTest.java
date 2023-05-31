package us.ihmc.valkyrie.controllerAPI;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.controllerAPI.EndToEndHandFingerTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.hands.athena.AthenaFingerControlParameters;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaJointName;
import us.ihmc.valkyrie.hands.athena.AthenaTrajectoryMessageConversion;
import valkyrie_msgs.msg.dds.AthenaTrajectoryMessage;

public class ValkyrieEndToEndAthenaTrajectoryMessageTest extends EndToEndHandFingerTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

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

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testCloseAndStopAndOpen() throws SimulationExceededMaximumTimeException
   {
      super.testCloseAndStopAndOpen();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testCloseAndOpenFingers() throws SimulationExceededMaximumTimeException
   {
      super.testCloseAndOpenFingers();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testClose() throws SimulationExceededMaximumTimeException
   {
      super.testClose();
   }

   @Override
   public AthenaTrajectoryMessage createTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration)
   {
      AthenaTrajectoryMessage message = new AthenaTrajectoryMessage();

      AthenaTrajectoryMessageConversion.convertHandConfiguration(robotSide, handConfiguration, message);

      return message;
   }

   @Override
   public void assertDesiredFingerJoint(RobotSide robotSide, HandConfiguration handConfiguration, double epsilon)
   {
      AthenaTrajectoryMessage athenaTrajectoryMessage = createTrajectoryMessage(robotSide, handConfiguration);

      int numberOfFingers = athenaTrajectoryMessage.getFingerMotorNames().size();

      for (int i = 0; i < numberOfFingers; i++)
      {
         double desiredPosition = getExpectedFingerMotorPosition(athenaTrajectoryMessage, i);

         double[] expectedHandJointPositions = getExpectedHandJointPosition(athenaTrajectoryMessage, i, desiredPosition);
         double[] handJointPositions = getCurrentHandJointPosition(athenaTrajectoryMessage, i);

         for (int j = 0; j < handJointPositions.length; j++)
         {
            assertEquals(expectedHandJointPositions[j], handJointPositions[j], epsilon);
         }
      }
   }

   private double getExpectedFingerMotorPosition(AthenaTrajectoryMessage athenaTrajectoryMessage, int indexOfFinger)
   {
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = athenaTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
      Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(indexOfFinger).getTrajectoryPoints();
      TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPoints.getLast();
      double desiredPosition = trajectoryPoint1DMessage.getPosition();

      return desiredPosition;
   }

   private double[] getExpectedHandJointPosition(AthenaTrajectoryMessage athenaTrajectoryMessage, int indexOfFinger, double desiredPosition)
   {
      AthenaFingerMotorName fingerMotorName = AthenaFingerMotorName.fromByte(athenaTrajectoryMessage.getFingerMotorNames().get(indexOfFinger));
      RobotSide robotSide = RobotSide.fromByte(athenaTrajectoryMessage.getRobotSide());

      List<? extends OneDoFJointReadOnly> handJoints = getAllHandJoints(robotSide, fingerMotorName);

      double[] handJointPositions = new double[handJoints.size()];
      for (int i = 0; i < handJoints.size(); i++)
      {
         AthenaJointName handJointName = fingerMotorName.getCorrespondingJointName(i);
         handJointPositions[i] = AthenaFingerControlParameters.getDesiredHandJoint(robotSide, handJointName, desiredPosition);
      }

      return handJointPositions;
   }

   private double[] getCurrentHandJointPosition(AthenaTrajectoryMessage athenaTrajectoryMessage, int indexOfFinger)
   {
      AthenaFingerMotorName fingerMotorName = AthenaFingerMotorName.fromByte(athenaTrajectoryMessage.getFingerMotorNames().get(indexOfFinger));
      RobotSide robotSide = RobotSide.fromByte(athenaTrajectoryMessage.getRobotSide());

      List<? extends OneDoFJointReadOnly> handJoints = getAllHandJoints(robotSide, fingerMotorName);

      double[] handJointPositions = new double[handJoints.size()];
      for (int i = 0; i < handJoints.size(); i++)
      {
         handJointPositions[i] = handJoints.get(i).getQ();
      }

      return handJointPositions;
   }

   private List<? extends OneDoFJointReadOnly> getAllHandJoints(RobotSide robotSide, AthenaFingerMotorName fingerMotorName)
   {
      List<OneDoFJointReadOnly> handJoints = new ArrayList<>();

      AthenaJointName firstHandJointName = fingerMotorName.getCorrespondingJointName(0);
      Robot simRobot = simulationTestHelper.getRobot();
      SimOneDoFJointBasics firstHandJoint = simRobot.getOneDoFJoint(firstHandJointName.getJointName(robotSide));

      if (fingerMotorName == AthenaFingerMotorName.ThumbMotorRoll)
         handJoints.add(firstHandJoint);
      else
         handJoints.addAll(SubtreeStreams.from(OneDoFJointReadOnly.class, firstHandJoint).collect(Collectors.toList()));

      return handJoints;
   }
}
