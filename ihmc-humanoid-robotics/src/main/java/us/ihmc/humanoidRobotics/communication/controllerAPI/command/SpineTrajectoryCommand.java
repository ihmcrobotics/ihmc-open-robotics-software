package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;

import java.util.Random;

public class SpineTrajectoryCommand implements Command<SpineTrajectoryCommand, SpineTrajectoryMessage>, EpsilonComparable<SpineTrajectoryCommand>
{
   private final JointspaceTrajectoryCommand jointspaceTrajectory;

   public SpineTrajectoryCommand()
   {
      jointspaceTrajectory = new JointspaceTrajectoryCommand();
   }

   public SpineTrajectoryCommand(Random random)
   {
      jointspaceTrajectory = new JointspaceTrajectoryCommand(random);
   }

   @Override
   public void clear()
   {
      jointspaceTrajectory.clear();
   }

   @Override
   public void set(SpineTrajectoryCommand other)
   {
      jointspaceTrajectory.set(other.jointspaceTrajectory);
   }

   @Override
   public void set(SpineTrajectoryMessage message)
   {
      jointspaceTrajectory.set(message.jointspaceTrajectory);
   }

   public JointspaceTrajectoryCommand getJointspaceTrajectory()
   {
      return jointspaceTrajectory;
   }

   @Override
   public boolean isCommandValid()
   {
      return jointspaceTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(SpineTrajectoryCommand other, double epsilon)
   {
      return jointspaceTrajectory.epsilonEquals(jointspaceTrajectory, epsilon);
   }

   @Override
   public Class<SpineTrajectoryMessage> getMessageClass()
   {
      return SpineTrajectoryMessage.class;
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      jointspaceTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      jointspaceTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return jointspaceTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return jointspaceTrajectory.getExecutionTime();
   }
}
