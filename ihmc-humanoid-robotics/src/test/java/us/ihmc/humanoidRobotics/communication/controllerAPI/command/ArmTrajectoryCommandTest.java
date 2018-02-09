package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTrajectoryCommandTest
{

   @Test(timeout = 30000, expected = IndexOutOfBoundsException.class)
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
      armTrajectoryCommand.getJointspaceTrajectory().getJointTrajectoryPoint(0, 0);
   }

   @Test(timeout = 30000)
   public void testIsCommandValid()
   {
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand(new Random());
      assertTrue(armTrajectoryCommand.isCommandValid());
      armTrajectoryCommand.clear();
      assertFalse(armTrajectoryCommand.isCommandValid());
   }

   @Test(timeout = 30000)
   public void testArmTrajectoryCommand()
   {
      new ArmTrajectoryCommand();
   }

   @Test(timeout = 30000)
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

   @Test(timeout = 30000)
   public void testSetRobotSide()
   {
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();
      assertNull(armTrajectoryCommand.getRobotSide());
      armTrajectoryCommand.setRobotSide(RobotSide.LEFT);
      assertEquals(RobotSide.LEFT, armTrajectoryCommand.getRobotSide());
   }

   @Test(timeout = 30000)
   public void testSetArmTrajectoryMessage()
   {
      Random random = new Random();
      ArmTrajectoryCommand armTrajectoryCommand = new ArmTrajectoryCommand();
      ArmTrajectoryMessage message = new ArmTrajectoryMessage(random);
      armTrajectoryCommand.set(message);

      assertEquals(message.getQueueingProperties().getExecutionDelayTime(), armTrajectoryCommand.getExecutionDelayTime(), 1e-9);
      assertEquals(message.getQueueingProperties().getExecutionMode(), armTrajectoryCommand.getJointspaceTrajectory().getExecutionMode());
      assertEquals(message.getJointspaceTrajectory().getNumberOfJoints(), armTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints());

      for (int i = 0; i < message.getJointspaceTrajectory().getNumberOfJoints(); i++)
      {
         int numberOfJointTrajectoryPoints = message.getJointspaceTrajectory().getNumberOfJointTrajectoryPoints(i);
         OneDoFJointTrajectoryCommand jointTrajectoryPointList = armTrajectoryCommand.getJointspaceTrajectory().getJointTrajectoryPointList(i);
         assertEquals(numberOfJointTrajectoryPoints, jointTrajectoryPointList.getNumberOfTrajectoryPoints());

         for (int j = 0; j < numberOfJointTrajectoryPoints; j++)
         {
            SimpleTrajectoryPoint1D trajectoryPoint = jointTrajectoryPointList.getTrajectoryPoint(j);
            TrajectoryPoint1DMessage jointTrajectoryPoint = message.getJointspaceTrajectory().getJointTrajectoryPoint(i, j);

            assertEquals(jointTrajectoryPoint.getPosition(), trajectoryPoint.getPosition(), 1e-9);
            assertEquals(jointTrajectoryPoint.getVelocity(), trajectoryPoint.getVelocity(), 1e-9);
            assertEquals(jointTrajectoryPoint.getTime(), trajectoryPoint.getTime(), 1e-9);
         }
      }
   }

   @Test(timeout = 30000)
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

         assertEquals(jointTrajectoryPointList.getCommandId(), otherJointTrajectoryPointList.getCommandId());
         assertEquals(jointTrajectoryPointList.getExecutionDelayTime(), otherJointTrajectoryPointList.getExecutionDelayTime(), 1e-8);
         assertEquals(jointTrajectoryPointList.getExecutionTime(), otherJointTrajectoryPointList.getExecutionTime(), 1e-8);
         assertEquals(jointTrajectoryPointList.getMessageClass(), otherJointTrajectoryPointList.getMessageClass());
         assertEquals(jointTrajectoryPointList.getNumberOfTrajectoryPoints(), otherJointTrajectoryPointList.getNumberOfTrajectoryPoints());
         assertEquals(jointTrajectoryPointList.getTrajectoryTime(), otherJointTrajectoryPointList.getTrajectoryTime(), 1e-8);
         assertEquals(jointTrajectoryPointList.getWeight(), otherJointTrajectoryPointList.getWeight(), 1e-8);
         assertTrue(jointTrajectoryPointList.getLastTrajectoryPoint().epsilonEquals(otherJointTrajectoryPointList.getLastTrajectoryPoint(), 1e-8));

         for (int j = 0; j < jointTrajectoryPointList.getNumberOfTrajectoryPoints(); j++)
         {
            SimpleTrajectoryPoint1D trajectoryPoint = jointTrajectoryPointList.getTrajectoryPoint(j);
            SimpleTrajectoryPoint1D otherTrajectoryPoint = otherJointTrajectoryPointList.getTrajectoryPoint(j);

            trajectoryPoint.epsilonEquals(otherTrajectoryPoint, 1e-8);
         }
      }
   }

   @Test(timeout = 30000)
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
