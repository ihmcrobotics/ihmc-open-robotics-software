package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.momentum.CenterOfMassTrajectoryMessage;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class CenterOfMassTrajectoryCommand implements Command<CenterOfMassTrajectoryCommand, CenterOfMassTrajectoryMessage>, FrameBasedCommand<CenterOfMassTrajectoryMessage>
{
   private final EuclideanTrajectoryControllerCommand euclideanTrajectory = new EuclideanTrajectoryControllerCommand();

   public CenterOfMassTrajectoryCommand()
   {
      euclideanTrajectory.clear();
   }

   public CenterOfMassTrajectoryCommand(CenterOfMassTrajectoryCommand other)
   {
      set(other);
   }

   @Override
   public void set(CenterOfMassTrajectoryMessage message)
   {
      euclideanTrajectory.set(message.euclideanTrajectory);
   }

   @Override
   public void set(CenterOfMassTrajectoryCommand other)
   {
      euclideanTrajectory.set(other.euclideanTrajectory);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, CenterOfMassTrajectoryMessage message)
   {
      euclideanTrajectory.set(resolver, message.euclideanTrajectory);
   }

   @Override
   public void clear()
   {
      euclideanTrajectory.clear();
   }

   public EuclideanTrajectoryControllerCommand getEuclideanTrajectory()
   {
      return euclideanTrajectory;
   }

   @Override
   public Class<CenterOfMassTrajectoryMessage> getMessageClass()
   {
      return CenterOfMassTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return euclideanTrajectory.isCommandValid();
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      euclideanTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      euclideanTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return euclideanTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return euclideanTrajectory.getExecutionTime();
   }
}
