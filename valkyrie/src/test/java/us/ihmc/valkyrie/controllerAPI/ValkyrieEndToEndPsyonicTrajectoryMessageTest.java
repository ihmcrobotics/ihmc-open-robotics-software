package us.ihmc.valkyrie.controllerAPI;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Arrays;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.controllerAPI.EndToEndHandFingerTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.hands.psyonic.PsyonicHandModel.PsyonicJointName;
import valkyrie_msgs.msg.dds.PsyonicTrajectoryMessage;

public class ValkyrieEndToEndPsyonicTrajectoryMessageTest extends EndToEndHandFingerTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.DUAL_PSYONIC);

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
   public PsyonicTrajectoryMessage createTrajectoryMessage(RobotSide robotSide, HandConfiguration handConfiguration)
   {
      PsyonicTrajectoryMessage message = new PsyonicTrajectoryMessage();

      convertHandConfiguration(robotSide, handConfiguration, message);

      return message;
   }

   @Override
   public void assertDesiredFingerJoint(RobotSide robotSide, HandConfiguration handConfiguration, double epsilon)
   {
      PsyonicTrajectoryMessage psyonicTrajectoryMessage = createTrajectoryMessage(robotSide, handConfiguration);

      int numberOfJoints = psyonicTrajectoryMessage.getJointNames().size();

      for (int i = 0; i < numberOfJoints; i++)
      {
         double desiredPosition = getExpectedPosition(psyonicTrajectoryMessage, i);
         double currentPosition = getCurrentPosition(psyonicTrajectoryMessage, i);
         assertEquals(desiredPosition, currentPosition, epsilon);
      }
   }

   private double getExpectedPosition(PsyonicTrajectoryMessage psyonicTrajectoryMessage, int jointIndex)
   {
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = psyonicTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
      Object<TrajectoryPoint1DMessage> trajectoryPoints = jointTrajectoryMessages.get(jointIndex).getTrajectoryPoints();
      TrajectoryPoint1DMessage trajectoryPoint1DMessage = trajectoryPoints.getLast();
      return trajectoryPoint1DMessage.getPosition();
   }

   private double getCurrentPosition(PsyonicTrajectoryMessage psyonicTrajectoryMessage, int jointIndex)
   {
      PsyonicJointName jointName = PsyonicJointName.fromByte(psyonicTrajectoryMessage.getJointNames().get(jointIndex));
      RobotSide robotSide = RobotSide.fromByte(psyonicTrajectoryMessage.getRobotSide());
      SimOneDoFJointBasics joint = simulationTestHelper.getRobot().getOneDoFJoint(jointName.getJointName(robotSide));
      return joint.getQ();
   }

   public static final void convertHandConfiguration(RobotSide robotSide, HandConfiguration desiredHandConfiguration, PsyonicTrajectoryMessage messageToPack)
   {
      messageToPack.setRobotSide(robotSide.toByte());
      convertHandConfiguration(desiredHandConfiguration, messageToPack);
   }

   public static final void convertHandConfiguration(HandConfiguration desiredHandConfiguration, PsyonicTrajectoryMessage messageToPack)
   {
      double trajectoryTime = 0.5;
      PsyonicJointName[] jointNames = null;
      double[] trajectoryTimes = null;
      double[] desiredJointPositions = null;

      switch (desiredHandConfiguration)
      {
         case STOP:
            break;

         case OPEN:
            jointNames = PsyonicJointName.values;
            trajectoryTimes = new double[jointNames.length];
            Arrays.fill(trajectoryTimes, trajectoryTime);
            desiredJointPositions = new double[jointNames.length];
            break;

         case CLOSE:
            jointNames = PsyonicJointName.values;
            trajectoryTimes = new double[jointNames.length];
            Arrays.fill(trajectoryTimes, trajectoryTime);
            desiredJointPositions = new double[] {-2, 1.3, 2, 2.1, 2, 2.1, 2, 2.1, 2, 2.1};
            break;

         case OPEN_FINGERS:
            jointNames = PsyonicJointName.fingerJoints;
            trajectoryTimes = new double[jointNames.length];
            Arrays.fill(trajectoryTimes, trajectoryTime);
            desiredJointPositions = new double[jointNames.length];
            break;

         case OPEN_THUMB:
            jointNames = PsyonicJointName.thumbJoints;
            trajectoryTimes = new double[jointNames.length];
            Arrays.fill(trajectoryTimes, trajectoryTime);
            desiredJointPositions = new double[jointNames.length];
            break;

         case CLOSE_FINGERS:
            jointNames = PsyonicJointName.fingerJoints;
            trajectoryTimes = new double[jointNames.length];
            Arrays.fill(trajectoryTimes, trajectoryTime);
            desiredJointPositions = new double[] {2, 2.1, 2, 2.1, 2, 2.1, 2, 2.1};
            break;

         case CLOSE_THUMB:
            jointNames = PsyonicJointName.thumbJoints;
            trajectoryTimes = new double[jointNames.length];
            Arrays.fill(trajectoryTimes, trajectoryTime);
            desiredJointPositions = new double[] {-2, 1.3};
            break;

         default:
            throw new IllegalArgumentException("Message conversion for the desired HandConfiguration is not implemented");
      }

      if (jointNames != null)
      {
         int dimension = jointNames.length;
         messageToPack.getJointspaceTrajectory().set(HumanoidMessageTools.createJointspaceTrajectoryMessage(trajectoryTimes, desiredJointPositions));
         for (int i = 0; i < dimension; i++)
         {
            messageToPack.getJointNames().add(jointNames[i].toByte());
         }
      }
   }
}
