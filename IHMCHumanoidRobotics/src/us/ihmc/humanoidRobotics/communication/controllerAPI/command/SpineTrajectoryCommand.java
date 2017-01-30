package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
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
      // TODO Auto-generated method stub

   }

   @Override
   public void set(SpineTrajectoryMessage message)
   {
      // TODO Auto-generated method stub

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
