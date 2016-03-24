package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmOneJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTrajectoryCommand implements Command<ArmTrajectoryCommand, ArmTrajectoryMessage>
{
   private final RecyclingArrayList<SimpleTrajectoryPoint1DList> jointTrajectoryInputs = new RecyclingArrayList<>(10, SimpleTrajectoryPoint1DList.class);
   private RobotSide robotSide;

   public ArmTrajectoryCommand()
   {
      clear();
   }

   public void clear()
   {
      clear(null);
   }

   public void clear(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      jointTrajectoryInputs.clear();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void addArmTrajectoryPoint(double trajectoryPointTime, double[] desiredJointPositions, double[] desiredJointVelocities)
   {
      MathTools.checkIfEqual(desiredJointPositions.length, desiredJointVelocities.length);
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

   @Override
   public void set(ArmTrajectoryMessage message)
   {
      set(message.getRobotSide(), message.getTrajectoryPointLists());
   }

   @Override
   public void set(ArmTrajectoryCommand other)
   {
      set(other.robotSide, other.getTrajectoryPointLists());
   }

   public void set(RobotSide robotSide, ArmOneJointTrajectoryMessage[] trajectoryPointListArray)
   {
      clear(robotSide);
      for (int i = 0; i < trajectoryPointListArray.length; i++)
      {
         SimpleTrajectoryPoint1DList simpleTrajectoryPoint1DList = jointTrajectoryInputs.add();
         ArmOneJointTrajectoryMessage armOneJointTrajectoryMessage = trajectoryPointListArray[i];
         armOneJointTrajectoryMessage.getTrajectoryPoints(simpleTrajectoryPoint1DList);
      }
   }

   public void set(RobotSide robotSide, RecyclingArrayList<SimpleTrajectoryPoint1DList> trajectoryPointListArray)
   {
      clear(robotSide);
      for (int i = 0; i < trajectoryPointListArray.size(); i++)
      {
         set(i, trajectoryPointListArray.get(i));
      }
   }

   public void set(int jointIndex, SimpleTrajectoryPoint1DList otherTrajectoryPointList)
   {
      SimpleTrajectoryPoint1DList thisJointTrajectoryPointList = jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex);
      thisJointTrajectoryPointList.set(otherTrajectoryPointList);
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

   public RecyclingArrayList<SimpleTrajectoryPoint1DList> getTrajectoryPointLists()
   {
      return jointTrajectoryInputs;
   }

   public SimpleTrajectoryPoint1D getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getTrajectoryPoint(trajectoryPointIndex);
   }

   public SimpleTrajectoryPoint1DList getJointTrajectoryPointList(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex);
   }

   @Override
   public Class<ArmTrajectoryMessage> getMessageClass()
   {
      return ArmTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && getNumberOfJoints() > 0;
   }
}
