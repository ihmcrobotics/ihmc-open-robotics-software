package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.commons.lists.RecyclingArrayList;

public final class JointspaceTrajectoryCommand extends QueueableCommand<JointspaceTrajectoryCommand, JointspaceTrajectoryMessage>
{
   private long sequenceId;
   private final RecyclingArrayList<OneDoFJointTrajectoryCommand> jointTrajectoryInputs = new RecyclingArrayList<>(10, OneDoFJointTrajectoryCommand.class);

   public JointspaceTrajectoryCommand()
   {
      clear();
   }

   public JointspaceTrajectoryCommand(Random random)
   {
      clear();
      int degreesOfFreedom = random.nextInt(10) + 1;
      for(int i = 0; i < degreesOfFreedom; i++)
      {
         OneDoFJointTrajectoryCommand oneDoFJointTrajectoryCommand = new OneDoFJointTrajectoryCommand(random);
         jointTrajectoryInputs.add().set(oneDoFJointTrajectoryCommand);
      }
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      clearQueuableCommandVariables();
      jointTrajectoryInputs.clear();
   }

   @Override
   public void set(JointspaceTrajectoryCommand other)
   {
      sequenceId = other.sequenceId;
      setQueueableCommandVariables(other);
      set(other.getTrajectoryPointLists());
   }

   @Override
   public void setFromMessage(JointspaceTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      setQueueableCommandVariables(message.getQueueingProperties());
      set(message.getJointTrajectoryMessages());
   }

   private void set(RecyclingArrayList<? extends OneDoFJointTrajectoryCommand> trajectoryPointListArray)
   {
      for (int i = 0; i < trajectoryPointListArray.size(); i++)
      {
         set(i, trajectoryPointListArray.get(i));
      }
   }

   private void set(List<OneDoFJointTrajectoryMessage> trajectoryPointListArray)
   {
      for (int i = 0; i < trajectoryPointListArray.size(); i++)
      {
         OneDoFJointTrajectoryCommand oneDoFJointTrajectoryCommand = jointTrajectoryInputs.add();
         OneDoFJointTrajectoryMessage oneJointTrajectoryMessage = trajectoryPointListArray.get(i);
         if (oneJointTrajectoryMessage != null)
         {
            oneDoFJointTrajectoryCommand.setFromMessage(oneJointTrajectoryMessage);
            oneDoFJointTrajectoryCommand.setWeight(oneJointTrajectoryMessage.getWeight());
         }
      }
   }

   private void set(int jointIndex, OneDoFJointTrajectoryCommand otherTrajectoryPointList)
   {
      OneDoFJointTrajectoryCommand thisJointTrajectoryPointList = jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex);
      thisJointTrajectoryPointList.set(otherTrajectoryPointList);
   }

   @Override
   public boolean isCommandValid()
   {
      return executionModeValid() && getNumberOfJoints() > 0;
   }

   public RecyclingArrayList<OneDoFJointTrajectoryCommand> getTrajectoryPointLists()
   {
      return jointTrajectoryInputs;
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryInputs.size();
   }

   public OneDoFTrajectoryPoint getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getTrajectoryPoint(trajectoryPointIndex);
   }

   public OneDoFJointTrajectoryCommand getJointTrajectoryPointList(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex);
   }

   public double getTrajectoryStartTime()
   {
      if (getNumberOfJoints() == 0)
         return Double.NaN;

      double startTime = Double.POSITIVE_INFINITY;

      for (int i = 0; i < jointTrajectoryInputs.size(); i++)
      {
         OneDoFJointTrajectoryCommand oneDoFJointTrajectoryCommand = jointTrajectoryInputs.get(i);
         if (oneDoFJointTrajectoryCommand.getNumberOfTrajectoryPoints() > 0)
            startTime = Math.min(startTime, oneDoFJointTrajectoryCommand.getTrajectoryPoint(0).getTime());
      }

      return startTime;
   }

   public double getTrajectoryEndTime()
   {
      if (getNumberOfJoints() == 0)
         return Double.NaN;

      double endTime = 0.0;

      for (int i = 0; i < jointTrajectoryInputs.size(); i++)
      {
         OneDoFJointTrajectoryCommand oneDoFJointTrajectoryCommand = jointTrajectoryInputs.get(i);
         if (oneDoFJointTrajectoryCommand.getNumberOfTrajectoryPoints() > 0)
            endTime = Math.max(endTime, oneDoFJointTrajectoryCommand.getLastTrajectoryPoint().getTime());
      }

      return endTime;
   }

   /** {@inheritDoc}} */
   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      for (int i = 0; i < jointTrajectoryInputs.size(); i++)
         jointTrajectoryInputs.get(i).addTimeOffset(timeOffsetToAdd);
   }

   @Override
   public boolean epsilonEquals(JointspaceTrajectoryCommand other, double epsilon)
   {
      if (this.jointTrajectoryInputs.size() != other.getTrajectoryPointLists().size())
      {
         return false;
      }

      for (int i = 0; i < this.jointTrajectoryInputs.size(); i++)
      {
         if (!this.jointTrajectoryInputs.get(i).epsilonEquals(other.getTrajectoryPointLists().get(i), epsilon))
         {
            return false;
         }
      }
      return super.epsilonEquals(other, epsilon);
   }

   @Override
   public Class<JointspaceTrajectoryMessage> getMessageClass()
   {
      return JointspaceTrajectoryMessage.class;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
