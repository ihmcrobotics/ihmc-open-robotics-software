package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.packets.momentum.CenterOfMassTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.momentum.TrajectoryPoint3D;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class CenterOfMassTrajectoryCommand extends QueueableCommand<CenterOfMassTrajectoryCommand, CenterOfMassTrajectoryMessage>
{
   private final RecyclingArrayList<TrajectoryPoint3D> comTrajectory = new RecyclingArrayList<>(10, TrajectoryPoint3D.class);

   public CenterOfMassTrajectoryCommand()
   {
      comTrajectory.clear();
   }

   public CenterOfMassTrajectoryCommand(CenterOfMassTrajectoryCommand other)
   {
      set(other);
   }

   @Override
   public void set(CenterOfMassTrajectoryMessage message)
   {
      clear();
      for (int i = 0; i < message.getNumberOfComTrajectoryPoints(); i++)
      {
         TrajectoryPoint3D trajectoryPoint = comTrajectory.add();
         trajectoryPoint.set(message.getComTrajectoryPoint(i));
      }
      setQueueableCommandVariables(message);
   }

   @Override
   public void set(CenterOfMassTrajectoryCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfComTrajectoryPoints(); i++)
      {
         TrajectoryPoint3D trajectoryPoint = comTrajectory.add();
         trajectoryPoint.set(other.getComTrajectoryPoint(i));
      }
      setQueueableCommandVariables(other);
   }

   public void addComTrajectoryPoint(TrajectoryPoint3D trajectoryPoint)
   {
      comTrajectory.add().set(trajectoryPoint);
   }

   public int getNumberOfComTrajectoryPoints()
   {
      return comTrajectory.size();
   }

   public TrajectoryPoint3D getComTrajectoryPoint(int i)
   {
      return comTrajectory.get(i);
   }

   @Override
   public void clear()
   {
      comTrajectory.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      for (int i = 0; i < comTrajectory.size(); i++)
      {
         TrajectoryPoint3D trajectoryPoint = comTrajectory.get(i);
         trajectoryPoint.setTime(trajectoryPoint.getTime() + timeOffset);
      }
   }

   @Override
   public Class<CenterOfMassTrajectoryMessage> getMessageClass()
   {
      return CenterOfMassTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return !comTrajectory.isEmpty();
   }
}
