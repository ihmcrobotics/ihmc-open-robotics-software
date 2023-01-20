package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyJointspaceTrajectoryMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;

public final class WholeBodyJointspaceTrajectoryCommand extends QueueableCommand<WholeBodyJointspaceTrajectoryCommand, WholeBodyJointspaceTrajectoryMessage>
{
   private long sequenceId;
   private final TIntArrayList jointHashCodes = new TIntArrayList();
   private final RecyclingArrayList<OneDoFJointTrajectoryCommand> jointTrajectoryInputs = new RecyclingArrayList<>(10, OneDoFJointTrajectoryCommand.class);

   public WholeBodyJointspaceTrajectoryCommand()
   {
      clear();
   }

   public WholeBodyJointspaceTrajectoryCommand(Random random)
   {
      clear();
      int degreesOfFreedom = random.nextInt(10) + 1;
      for (int i = 0; i < degreesOfFreedom; i++)
      {
         jointHashCodes.add(random.nextInt());
         OneDoFJointTrajectoryCommand oneDoFJointTrajectoryCommand = new OneDoFJointTrajectoryCommand(random);
         jointTrajectoryInputs.add().set(oneDoFJointTrajectoryCommand);
      }
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      clearQueuableCommandVariables();
      jointHashCodes.reset();
      jointTrajectoryInputs.clear();
   }

   @Override
   public void set(WholeBodyJointspaceTrajectoryCommand other)
   {
      clear();
      sequenceId = other.sequenceId;
      setQueueableCommandVariables(other);

      for (int i = 0; i < other.jointHashCodes.size(); i++)
      {
         jointHashCodes.add(other.jointHashCodes.get(i));
      }

      for (int i = 0; i < other.jointTrajectoryInputs.size(); i++)
      {
         jointTrajectoryInputs.add().set(other.jointTrajectoryInputs.get(i));
      }
   }

   @Override
   public void setFromMessage(WholeBodyJointspaceTrajectoryMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();
      setQueueableCommandVariables(message.getQueueingProperties());
      List<OneDoFJointTrajectoryMessage> trajectoryPointListArray = message.getJointTrajectoryMessages();

      for (int i = 0; i < message.getJointHashCodes().size(); i++)
      {
         jointHashCodes.add(message.getJointHashCodes().get(i));
      }

      for (int i = 0; i < trajectoryPointListArray.size(); i++)
      {
         jointTrajectoryInputs.add().setFromMessage(trajectoryPointListArray.get(i));
      }
   }

   @Override
   public boolean isCommandValid()
   {
      return executionModeValid() && getNumberOfJoints() > 0 && jointHashCodes.size() == jointTrajectoryInputs.size();
   }

   public RecyclingArrayList<OneDoFJointTrajectoryCommand> getTrajectoryPointLists()
   {
      return jointTrajectoryInputs;
   }

   public int getNumberOfJoints()
   {
      return jointTrajectoryInputs.size();
   }

   public int getJointHashCode(int jointIndex)
   {
      return jointHashCodes.get(jointIndex);
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
   public boolean epsilonEquals(WholeBodyJointspaceTrajectoryCommand other, double epsilon)
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
   public Class<WholeBodyJointspaceTrajectoryMessage> getMessageClass()
   {
      return WholeBodyJointspaceTrajectoryMessage.class;
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
