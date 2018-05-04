package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import controller_msgs.msg.dds.NeckTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class NeckTrajectoryCommand implements Command<NeckTrajectoryCommand, NeckTrajectoryMessage>, EpsilonComparable<NeckTrajectoryCommand>
{
   private final JointspaceTrajectoryCommand jointspaceTrajectory;

   public NeckTrajectoryCommand()
   {
      jointspaceTrajectory = new JointspaceTrajectoryCommand();
   }

   public NeckTrajectoryCommand(Random random)
   {
      jointspaceTrajectory = new JointspaceTrajectoryCommand(random);
   }

   @Override
   public void clear()
   {
      jointspaceTrajectory.clear();
   }

   @Override
   public void set(NeckTrajectoryCommand other)
   {
      jointspaceTrajectory.set(other.jointspaceTrajectory);
   }

   @Override
   public void set(NeckTrajectoryMessage message)
   {
      jointspaceTrajectory.set(message.getJointspaceTrajectory());
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
   public boolean epsilonEquals(NeckTrajectoryCommand other, double epsilon)
   {
      return jointspaceTrajectory.epsilonEquals(other.jointspaceTrajectory, epsilon);
   }

   @Override
   public Class<NeckTrajectoryMessage> getMessageClass()
   {
      return NeckTrajectoryMessage.class;
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
