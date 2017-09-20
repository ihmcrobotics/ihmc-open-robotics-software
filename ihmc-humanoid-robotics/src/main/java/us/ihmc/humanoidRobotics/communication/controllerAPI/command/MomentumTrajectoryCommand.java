package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.packets.momentum.MomentumTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class MomentumTrajectoryCommand extends QueueableCommand<MomentumTrajectoryCommand, MomentumTrajectoryMessage>
{
   private final RecyclingArrayList<TrajectoryPoint3D> angularMomentumTrajectory = new RecyclingArrayList<>(10, TrajectoryPoint3D.class);

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
      clear();
      for (int i = 0; i < message.getNumberOfAngularMomentumTrajectoryPoints(); i++)
      {
         TrajectoryPoint3D trajectoryPoint = angularMomentumTrajectory.add();
         trajectoryPoint.set(message.getAngularMomentumTrajectoryPoint(i));
      }
      setQueueableCommandVariables(message);
   }

   @Override
   public void set(MomentumTrajectoryCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfAngularMomentumTrajectoryPoints(); i++)
      {
         TrajectoryPoint3D trajectoryPoint = angularMomentumTrajectory.add();
         trajectoryPoint.set(other.getAngularMomentumTrajectoryPoint(i));
      }
      setQueueableCommandVariables(other);
   }

   public void addAngularMomentumTrajectoryPoint(TrajectoryPoint3D trajectoryPoint)
   {
      angularMomentumTrajectory.add().set(trajectoryPoint);
   }

   public int getNumberOfAngularMomentumTrajectoryPoints()
   {
      return angularMomentumTrajectory.size();
   }

   public TrajectoryPoint3D getAngularMomentumTrajectoryPoint(int i)
   {
      return angularMomentumTrajectory.get(i);
   }

   @Override
   public void clear()
   {
      angularMomentumTrajectory.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      for (int i = 0; i < angularMomentumTrajectory.size(); i++)
      {
         TrajectoryPoint3D trajectoryPoint = angularMomentumTrajectory.get(i);
         trajectoryPoint.setTime(trajectoryPoint.getTime() + timeOffset);
      }
   }

   @Override
   public Class<MomentumTrajectoryMessage> getMessageClass()
   {
      return MomentumTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return !angularMomentumTrajectory.isEmpty();
   }

}
