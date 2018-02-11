package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.momentum.MomentumTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class MomentumTrajectoryCommand implements Command<MomentumTrajectoryCommand, MomentumTrajectoryMessage>, FrameBasedCommand<MomentumTrajectoryMessage>
{
   private final EuclideanTrajectoryControllerCommand angularMomentumTrajectory = new EuclideanTrajectoryControllerCommand();

   public MomentumTrajectoryCommand()
   {
      angularMomentumTrajectory.clear();
   }

   public MomentumTrajectoryCommand(MomentumTrajectoryCommand other)
   {
      set(other);
   }

   @Override
   public void set(MomentumTrajectoryMessage message)
   {
      angularMomentumTrajectory.set(message.angularMomentumTrajectory);
   }
   
   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, MomentumTrajectoryMessage message)
   {
      angularMomentumTrajectory.set(resolver, message.angularMomentumTrajectory);
   }

   @Override
   public void set(MomentumTrajectoryCommand other)
   {
      angularMomentumTrajectory.set(other.angularMomentumTrajectory);
   }

   @Override
   public void clear()
   {
      angularMomentumTrajectory.clear();
   }

   public EuclideanTrajectoryControllerCommand getAngularMomentumTrajectory()
   {
      return angularMomentumTrajectory;
   }

   @Override
   public Class<MomentumTrajectoryMessage> getMessageClass()
   {
      return MomentumTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return angularMomentumTrajectory.isCommandValid();
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      angularMomentumTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      angularMomentumTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return angularMomentumTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return angularMomentumTrajectory.getExecutionTime();
   }
}
