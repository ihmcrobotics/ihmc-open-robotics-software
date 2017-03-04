package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.robotSide.RobotSide;

public class NeckTrajectoryCommand implements Command<NeckTrajectoryCommand, NeckTrajectoryMessage>
{
   private final RecyclingArrayList<OneDoFJointTrajectoryCommand> jointTrajectoryInputs = new RecyclingArrayList<>(10, OneDoFJointTrajectoryCommand.class);

   public NeckTrajectoryCommand()
   {
      clear();
   }

   public void clear()
   {
      jointTrajectoryInputs.clear();
   }

   public void addNeckTrajectoryPoint(double trajectoryPointTime, double[] desiredJointPositions, double[] desiredJointVelocities)
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
      clear();
      for (int jointIndex = 0; jointIndex < desiredJointPositions.length; jointIndex++)
      {
         SimpleTrajectoryPoint1DList jointTrajectoryInput = jointTrajectoryInputs.add();
         jointTrajectoryInput.clear();
         jointTrajectoryInput.addTrajectoryPoint(trajectoryTime, desiredJointPositions[jointIndex], 0.0);
      }
   }

   @Override
   public void set(NeckTrajectoryMessage message)
   {
      set(message.getTrajectoryPointLists());
   }

   @Override
   public void set(NeckTrajectoryCommand other)
   {
      set(other.getTrajectoryPointLists());
   }

   public void set(OneDoFJointTrajectoryMessage[] trajectoryPointListArray)
   {
      clear();
      for (int i = 0; i < trajectoryPointListArray.length; i++)
      {
         SimpleTrajectoryPoint1DList simpleTrajectoryPoint1DList = jointTrajectoryInputs.add();
         OneDoFJointTrajectoryMessage oneJointTrajectoryMessage = trajectoryPointListArray[i];
         oneJointTrajectoryMessage.getTrajectoryPoints(simpleTrajectoryPoint1DList);
      }
   }

   public void set(RecyclingArrayList<? extends SimpleTrajectoryPoint1DList> trajectoryPointListArray)
   {
      clear();
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

   public RecyclingArrayList<OneDoFJointTrajectoryCommand> getTrajectoryPointLists()
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
   public Class<NeckTrajectoryMessage> getMessageClass()
   {
      return NeckTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfJoints() > 0;
   }
}
