package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.RandomHumanoidMessages;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTrajectoryCommandTest
{

   @Test
   public void testClear()
   {
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();
      armTrajectoryCommand.clear();
      assertNull(armTrajectoryCommand.getRobotSide());
      assertEquals(Packet.VALID_MESSAGE_DEFAULT_ID, armTrajectoryCommand.getJointspaceTrajectory().getCommandId());
      assertEquals(0.0, armTrajectoryCommand.getExecutionDelayTime(), 1e-9);
      assertEquals(ExecutionMode.OVERRIDE, armTrajectoryCommand.getJointspaceTrajectory().getExecutionMode());
      assertEquals(0.0, armTrajectoryCommand.getExecutionTime(), 1e-9);
      assertEquals(0, armTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints());
      assertEquals(Packet.INVALID_MESSAGE_ID, armTrajectoryCommand.getJointspaceTrajectory().getPreviousCommandId());
      assertEquals(0, armTrajectoryCommand.getJointspaceTrajectory().getTrajectoryPointLists().size());
      assertThrows(IndexOutOfBoundsException.class, () -> armTrajectoryCommand.getJointspaceTrajectory().getJointTrajectoryPoint(0, 0));
   }

   @Test
   public void testIsCommandValid()
   {
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand(new Random());
      assertTrue(armTrajectoryCommand.isCommandValid());
      armTrajectoryCommand.clear();
      assertFalse(armTrajectoryCommand.isCommandValid());
   }

   @Test
   public void testArmTrajectoryCommand()
   {
      new ArmTrajectoryCommand();
   }

   @Test
   public void testClearRobotSide()
   {
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();
      assertNull(armTrajectoryCommand.getRobotSide());
      armTrajectoryCommand.clear(RobotSide.LEFT);
      assertEquals(RobotSide.LEFT, armTrajectoryCommand.getRobotSide());

      assertEquals(Packet.VALID_MESSAGE_DEFAULT_ID, armTrajectoryCommand.getJointspaceTrajectory().getCommandId());
      assertEquals(0.0, armTrajectoryCommand.getExecutionDelayTime(), 1e-9);
      assertEquals(ExecutionMode.OVERRIDE, armTrajectoryCommand.getJointspaceTrajectory().getExecutionMode());
      assertEquals(0.0, armTrajectoryCommand.getExecutionTime(), 1e-9);
      assertEquals(0, armTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints());
      assertEquals(Packet.INVALID_MESSAGE_ID, armTrajectoryCommand.getJointspaceTrajectory().getPreviousCommandId());
      assertEquals(0, armTrajectoryCommand.getJointspaceTrajectory().getTrajectoryPointLists().size());
   }

   @Test
   public void testSetRobotSide()
   {
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();
      assertNull(armTrajectoryCommand.getRobotSide());
      armTrajectoryCommand.setRobotSide(RobotSide.LEFT);
      assertEquals(RobotSide.LEFT, armTrajectoryCommand.getRobotSide());
   }

   @Test
   public void testSetArmTrajectoryMessage()
   {
      Random random = new Random();
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();
      ArmTrajectoryMessage message = RandomHumanoidMessages.nextArmTrajectoryMessage(random);
      armTrajectoryCommand.setFromMessage(message);

      assertEquals(message.getJointspaceTrajectory().getQueueingProperties().getExecutionDelayTime(), armTrajectoryCommand.getExecutionDelayTime(), 1e-9);
      assertEquals(ExecutionMode.fromByte(message.getJointspaceTrajectory().getQueueingProperties().getExecutionMode()), armTrajectoryCommand.getJointspaceTrajectory().getExecutionMode());
      assertEquals(message.getJointspaceTrajectory().getJointTrajectoryMessages().size(), armTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints());

      for (int i = 0; i < message.getJointspaceTrajectory().getJointTrajectoryMessages().size(); i++)
      {
         int numberOfJointTrajectoryPoints = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(i).getTrajectoryPoints().size();
         OneDoFJointTrajectoryCommand jointTrajectoryPointList = armTrajectoryCommand.getJointspaceTrajectory().getJointTrajectoryPointList(i);
         assertEquals(numberOfJointTrajectoryPoints, jointTrajectoryPointList.getNumberOfTrajectoryPoints());

         for (int j = 0; j < numberOfJointTrajectoryPoints; j++)
         {
            OneDoFTrajectoryPoint trajectoryPoint = jointTrajectoryPointList.getTrajectoryPoint(j);
            TrajectoryPoint1DMessage jointTrajectoryPoint = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(i).getTrajectoryPoints().get(j);

            assertEquals(jointTrajectoryPoint.getPosition(), trajectoryPoint.getPosition(), 1e-9);
            assertEquals(jointTrajectoryPoint.getVelocity(), trajectoryPoint.getVelocity(), 1e-9);
            assertEquals(jointTrajectoryPoint.getTime(), trajectoryPoint.getTime(), 1e-9);
         }
      }
   }

   @Test
   public void testSetArmTrajectoryCommand()
   {
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand(new Random());
      ArmTrajectoryCommand otherArmTrajectoryCommand = new ArmTrajectoryCommand();
      armTrajectoryCommand.set(otherArmTrajectoryCommand);
      armTrajectoryCommand.epsilonEquals(otherArmTrajectoryCommand, 1e-8);

      assertEquals(armTrajectoryCommand.getJointspaceTrajectory().getCommandId(), otherArmTrajectoryCommand.getJointspaceTrajectory().getCommandId());
      assertEquals(armTrajectoryCommand.getExecutionDelayTime(), otherArmTrajectoryCommand.getExecutionDelayTime(), 1e-9);
      assertEquals(armTrajectoryCommand.getJointspaceTrajectory().getExecutionMode(), otherArmTrajectoryCommand.getJointspaceTrajectory().getExecutionMode());
      assertEquals(armTrajectoryCommand.getExecutionTime(), otherArmTrajectoryCommand.getExecutionTime(), 1e-9);
      assertEquals(armTrajectoryCommand.getMessageClass(), otherArmTrajectoryCommand.getMessageClass());
      assertEquals(armTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints(), otherArmTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints());
      assertEquals(armTrajectoryCommand.getJointspaceTrajectory().getPreviousCommandId(), otherArmTrajectoryCommand.getJointspaceTrajectory().getPreviousCommandId());
      assertEquals(armTrajectoryCommand.getRobotSide(), otherArmTrajectoryCommand.getRobotSide());

      for (int i = 0; i < armTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints(); i++)
      {
         OneDoFJointTrajectoryCommand jointTrajectoryPointList = armTrajectoryCommand.getJointspaceTrajectory().getJointTrajectoryPointList(i);
         OneDoFJointTrajectoryCommand otherJointTrajectoryPointList = otherArmTrajectoryCommand.getJointspaceTrajectory().getJointTrajectoryPointList(i);

         assertEquals(jointTrajectoryPointList.getExecutionDelayTime(), otherJointTrajectoryPointList.getExecutionDelayTime(), 1e-8);
         assertEquals(jointTrajectoryPointList.getExecutionTime(), otherJointTrajectoryPointList.getExecutionTime(), 1e-8);
         assertEquals(jointTrajectoryPointList.getMessageClass(), otherJointTrajectoryPointList.getMessageClass());
         assertEquals(jointTrajectoryPointList.getNumberOfTrajectoryPoints(), otherJointTrajectoryPointList.getNumberOfTrajectoryPoints());
         assertEquals(jointTrajectoryPointList.getTrajectoryTime(), otherJointTrajectoryPointList.getTrajectoryTime(), 1e-8);
         assertEquals(jointTrajectoryPointList.getWeight(), otherJointTrajectoryPointList.getWeight(), 1e-8);
         assertTrue(jointTrajectoryPointList.getLastTrajectoryPoint().epsilonEquals(otherJointTrajectoryPointList.getLastTrajectoryPoint(), 1e-8));

         for (int j = 0; j < jointTrajectoryPointList.getNumberOfTrajectoryPoints(); j++)
         {
            OneDoFTrajectoryPoint trajectoryPoint = jointTrajectoryPointList.getTrajectoryPoint(j);
            OneDoFTrajectoryPoint otherTrajectoryPoint = otherJointTrajectoryPointList.getTrajectoryPoint(j);

            trajectoryPoint.epsilonEquals(otherTrajectoryPoint, 1e-8);
         }
      }
   }

   @Test
   public void testGetMessageClass()
   {
      ArmTrajectoryCommand otherArmTrajectoryCommand = new ArmTrajectoryCommand();
      assertEquals(ArmTrajectoryMessage.class, otherArmTrajectoryCommand.getMessageClass());
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();
      mutationTestFacilitator.addClassesToMutate(new Class[] {ArmTrajectoryCommand.class, JointspaceTrajectoryCommand.class, QueueableCommand.class});
      mutationTestFacilitator.addTestClassesToRun(ArmTrajectoryCommandTest.class);
      mutationTestFacilitator.doMutationTest();
      mutationTestFacilitator.openResultInBrowser();
   }
}
