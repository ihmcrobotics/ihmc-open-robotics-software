package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;

public class CrocoddylSolverTrajectoryCommand implements Command<CrocoddylSolverTrajectoryCommand, CrocoddylSolverTrajectoryMessage>
{
   private int sequenceId = -1;
   private final RecyclingArrayList<CrocoddylTimeIntervalCommand> intervals = new RecyclingArrayList<>(CrocoddylTimeIntervalCommand::new);
   private final RecyclingArrayList<CrocoddylStateCommand> stateTrajectory = new RecyclingArrayList<>(CrocoddylStateCommand::new);
   private final RecyclingArrayList<CrocoddylControlCommand> controlTrajectory = new RecyclingArrayList<>(CrocoddylControlCommand::new);

   @Override
   public void clear()
   {
      sequenceId = -1;
      this.intervals.clear();
      stateTrajectory.clear();
      controlTrajectory.clear();
   }

   @Override
   public void setFromMessage(CrocoddylSolverTrajectoryMessage message)
   {
      clear();

      this.sequenceId = (int) message.getUniqueId();

      for (int i = 0; i < message.getIntervals().size(); i++)
      {
         this.intervals.add().setFromMessage(message.getIntervals().get(i));
         this.stateTrajectory.add().setFromMessage(message.getStateTrajectory().get(i));
         this.controlTrajectory.add().setFromMessage(message.getControlTrajectory().get(i));
      }
   }

   @Override
   public Class<CrocoddylSolverTrajectoryMessage> getMessageClass()
   {
      return CrocoddylSolverTrajectoryMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(CrocoddylSolverTrajectoryCommand other)
   {
      clear();

      this.sequenceId = other.sequenceId;
      for (int i = 0; i < other.intervals.size(); i++)
      {
         this.intervals.add().set(other.intervals.get(i));
         this.stateTrajectory.add().set(other.stateTrajectory.get(i));
         this.controlTrajectory.add().set(other.controlTrajectory.get(i));
      }
   }

   public int getNumberOfKnots()
   {
      return intervals.size();
   }

   public CrocoddylTimeIntervalCommand getTimeInterval(int knot)
   {
      return intervals.get(knot);
   }

   public CrocoddylStateCommand getState(int knot)
   {
      return stateTrajectory.get(knot);
   }

   public CrocoddylControlCommand getControl(int knot)
   {
      return controlTrajectory.get(knot);
   }
}
