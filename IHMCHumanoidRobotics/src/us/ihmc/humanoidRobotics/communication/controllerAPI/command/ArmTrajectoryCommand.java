package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTrajectoryCommand implements Command<ArmTrajectoryCommand, ArmTrajectoryMessage>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   private final RecyclingArrayList<OneDoFJointTrajectoryCommand> jointTrajectoryInputs = new RecyclingArrayList<>(10, OneDoFJointTrajectoryCommand.class);
   private RobotSide robotSide;
   private ExecutionMode executionMode;
   private long previousCommandId = Packet.INVALID_MESSAGE_ID;

   public ArmTrajectoryCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      clear(null);
   }

   public void clear(RobotSide robotSide)
   {
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      jointTrajectoryInputs.clear();
      this.robotSide = robotSide;
      executionMode = null;
      previousCommandId = Packet.INVALID_MESSAGE_ID;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void addArmTrajectoryPoint(double trajectoryPointTime, double[] desiredJointPositions, double[] desiredJointVelocities)
   {
      MathTools.checkEquals(desiredJointPositions.length, desiredJointVelocities.length);
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         SimpleTrajectoryPoint1DList jointTrajectoryInput = jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex);
         jointTrajectoryInput.addTrajectoryPoint(trajectoryPointTime, desiredJointPositions[jointIndex], desiredJointVelocities[jointIndex]);
      }
   }

   public void set(RobotSide robotSide, double trajectoryTime, double[] desiredJointPositions)
   {
      clear(robotSide);
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         SimpleTrajectoryPoint1DList jointTrajectoryInput = jointTrajectoryInputs.add();
         jointTrajectoryInput.clear();
         jointTrajectoryInput.addTrajectoryPoint(trajectoryTime, desiredJointPositions[jointIndex], 0.0);
      }
   }

   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
   }

   public void setExecutionMode(ExecutionMode executionMode)
   {
      this.executionMode = executionMode;
   }

   public void setPreviousCommandId(long previousCommandId)
   {
      this.previousCommandId = previousCommandId;
   }

   @Override
   public void set(ArmTrajectoryMessage message)
   {
      set(message.getRobotSide(), message.getTrajectoryPointLists());
      commandId = message.getUniqueId();
      executionMode = message.getExecutionMode();
      previousCommandId = message.getPreviousMessageId();
   }

   @Override
   public void set(ArmTrajectoryCommand other)
   {
      set(other.robotSide, other.getTrajectoryPointLists());
      commandId = other.commandId;
      executionMode = other.executionMode;
      previousCommandId = other.previousCommandId;
   }

   public void set(RobotSide robotSide, OneDoFJointTrajectoryMessage[] trajectoryPointListArray)
   {
      clear(robotSide);
      for (int i = 0; i < trajectoryPointListArray.length; i++)
      {
         SimpleTrajectoryPoint1DList simpleTrajectoryPoint1DList = jointTrajectoryInputs.add();
         OneDoFJointTrajectoryMessage armOneJointTrajectoryMessage = trajectoryPointListArray[i];
         armOneJointTrajectoryMessage.getTrajectoryPoints(simpleTrajectoryPoint1DList);
      }
   }

   public void set(RobotSide robotSide, RecyclingArrayList<? extends SimpleTrajectoryPoint1DList> trajectoryPointListArray)
   {
      clear(robotSide);
      for (int i = 0; i < trajectoryPointListArray.size(); i++)
         set(i, trajectoryPointListArray.get(i));
   }

   public void set(int jointIndex, SimpleTrajectoryPoint1DList otherTrajectoryPointList)
   {
      SimpleTrajectoryPoint1DList thisJointTrajectoryPointList = jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex);
      thisJointTrajectoryPointList.set(otherTrajectoryPointList);
   }

   public void appendTrajectoryPoint(int jointIndex, SimpleTrajectoryPoint1D trajectoryPoint)
   {
      jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex).addTrajectoryPoint(trajectoryPoint);
   }

   public long getCommandId()
   {
      return commandId;
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryInputs.size();
   }

   public int getNumberOfJointTrajectoryPoints(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getNumberOfTrajectoryPoints();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public RecyclingArrayList<OneDoFJointTrajectoryCommand> getTrajectoryPointLists()
   {
      return jointTrajectoryInputs;
   }

   public SimpleTrajectoryPoint1D getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getTrajectoryPoint(trajectoryPointIndex);
   }

   public OneDoFJointTrajectoryCommand getJointTrajectoryPointList(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex);
   }

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
   }

   public long getPreviousCommandId()
   {
      return previousCommandId;
   }

   @Override
   public Class<ArmTrajectoryMessage> getMessageClass()
   {
      return ArmTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && executionMode != null && getNumberOfJoints() > 0;
   }
}
