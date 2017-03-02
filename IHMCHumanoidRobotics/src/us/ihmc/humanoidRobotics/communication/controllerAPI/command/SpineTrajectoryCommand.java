package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class SpineTrajectoryCommand implements Command<SpineTrajectoryCommand, SpineTrajectoryMessage>
{
   private final RecyclingArrayList<OneDoFJointTrajectoryCommand> jointTrajectoryInputs = new RecyclingArrayList<>(10, OneDoFJointTrajectoryCommand.class);

   public SpineTrajectoryCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      jointTrajectoryInputs.clear();
   }

   @Override
   public void set(SpineTrajectoryCommand other)
   {
      set(other.getTrajectoryPointLists());
   }

   @Override
   public void set(SpineTrajectoryMessage message)
   {
      set(message.getTrajectoryPointsLists());
   }

   public void set(RecyclingArrayList<? extends SimpleTrajectoryPoint1DList> trajectoryPointListArray)
   {
      clear();
      for (int i = 0; i < trajectoryPointListArray.size(); i++)
      {
         set(i, trajectoryPointListArray.get(i));
      }
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

   public void set(int jointIndex, SimpleTrajectoryPoint1DList otherTrajectoryPointList)
   {
      SimpleTrajectoryPoint1DList thisJointTrajectoryPointList = jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex);
      thisJointTrajectoryPointList.set(otherTrajectoryPointList);
   }

   @Override
   public Class<SpineTrajectoryMessage> getMessageClass()
   {
      return SpineTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return getNumberOfJoints() > 0;
   }

   public RecyclingArrayList<OneDoFJointTrajectoryCommand> getTrajectoryPointLists()
   {
      return jointTrajectoryInputs;
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryInputs.size();
   }

   public SimpleTrajectoryPoint1D getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getTrajectoryPoint(trajectoryPointIndex);
   }

   public SimpleTrajectoryPoint1DList getJointTrajectoryPointList(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex);
   }

}
