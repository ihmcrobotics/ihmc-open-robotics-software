package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.packets.AbstractJointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3HybridJointSpaceTaskSpaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;

public abstract class HybridSE3JointspaceTaskspaceTrajectoryCommand<
   C extends HybridSE3JointspaceTaskspaceTrajectoryCommand<C, M, TC, TM, JC, JM>,
   M extends AbstractSE3HybridJointSpaceTaskSpaceTrajectoryMessage<M, TM, JM>,
   TC extends SE3TrajectoryControllerCommand<TC, TM>,
   TM extends AbstractSE3TrajectoryMessage<TM>,
   JC extends JointspaceTrajectoryCommand<JC, JM>,
   JM extends AbstractJointspaceTrajectoryMessage<JM>
> extends QueueableCommand<C, M>
{

   protected JointspaceTrajectoryCommand<JC, JM> jointspaceTrajectoryCommand;
   protected SE3TrajectoryControllerCommand<TC, TM> taskspaceTrajectoryCommand;
   
   public HybridSE3JointspaceTaskspaceTrajectoryCommand()
   {
      
   }
   
   public HybridSE3JointspaceTaskspaceTrajectoryCommand(JointspaceTrajectoryCommand<JC, JM> jointspaceTrajectoryCommand, SE3TrajectoryControllerCommand<TC, TM> taskspaceTrajectoryCommand)
   {
      this.setJointspaceTrajectoryCommand(jointspaceTrajectoryCommand);
      this.setTaskspaceTrajectoryCommand(taskspaceTrajectoryCommand);
   }

   @Override
   public void clear()
   {
      getJointspaceTrajectoryCommand().clear();
      getTaskspaceTrajectoryCommand().clear();
   }

   @Override
   public void set(C other)
   {
      this.setJointspaceTrajectoryCommand(jointspaceTrajectoryCommand);
      this.setTaskspaceTrajectoryCommand(taskspaceTrajectoryCommand);
   }

   @Override
   public void set(M message)
   {
      jointspaceTrajectoryCommand.set(message.getJointspaceTrajectoryMessage());
      taskspaceTrajectoryCommand.set(message.getTaskspaceTrajectoryMessage());
   }

   @Override
   public boolean isCommandValid()
   {
      return getJointspaceTrajectoryCommand().isCommandValid() && getTaskspaceTrajectoryCommand().isCommandValid();
   }

   /** {@inheritDoc}} */
   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      getTaskspaceTrajectoryCommand().addTimeOffset(timeOffsetToAdd);
      getJointspaceTrajectoryCommand().addTimeOffset(timeOffsetToAdd);
   }

   public JointspaceTrajectoryCommand<JC, JM> getJointspaceTrajectoryCommand()
   {
      return jointspaceTrajectoryCommand;
   }

   public void setJointspaceTrajectoryCommand(JointspaceTrajectoryCommand<JC, JM> jointspaceTrajectoryCommand)
   {
      this.jointspaceTrajectoryCommand = jointspaceTrajectoryCommand;
   }

   public SE3TrajectoryControllerCommand<TC, TM> getTaskspaceTrajectoryCommand()
   {
      return taskspaceTrajectoryCommand;
   }

   public void setTaskspaceTrajectoryCommand(SE3TrajectoryControllerCommand<TC, TM> taskspaceTrajectoryCommand)
   {
      this.taskspaceTrajectoryCommand = taskspaceTrajectoryCommand;
   }
}
