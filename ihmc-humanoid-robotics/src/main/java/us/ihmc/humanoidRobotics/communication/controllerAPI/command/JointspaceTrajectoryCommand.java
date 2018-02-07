package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.packets.JointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;

public final class JointspaceTrajectoryCommand extends QueueableCommand<JointspaceTrajectoryCommand, JointspaceTrajectoryMessage>
{
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
      clearQueuableCommandVariables();
      jointTrajectoryInputs.clear();
   }

   @Override
   public void set(JointspaceTrajectoryCommand other)
   {
      setQueueableCommandVariables(other);
      set(other.getTrajectoryPointLists());
   }

   @Override
   public void set(JointspaceTrajectoryMessage message)
   {
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
      set(message.getTrajectoryPointLists());
   }

   private void set(RecyclingArrayList<? extends OneDoFJointTrajectoryCommand> trajectoryPointListArray)
   {
      for (int i = 0; i < trajectoryPointListArray.size(); i++)
      {
         set(i, trajectoryPointListArray.get(i));
      }
   }

   private void set(OneDoFJointTrajectoryMessage[] trajectoryPointListArray)
   {
      for (int i = 0; i < trajectoryPointListArray.length; i++)
      {
         OneDoFJointTrajectoryCommand oneDoFJointTrajectoryCommand = jointTrajectoryInputs.add();
         OneDoFJointTrajectoryMessage oneJointTrajectoryMessage = trajectoryPointListArray[i];
         if (oneJointTrajectoryMessage != null)
         {
            oneJointTrajectoryMessage.getTrajectoryPoints(oneDoFJointTrajectoryCommand);
            oneDoFJointTrajectoryCommand.setWeight(oneJointTrajectoryMessage.getWeight());
         }
         else
         {
            oneDoFJointTrajectoryCommand.clear();
            oneDoFJointTrajectoryCommand.setWeight(0.0);
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

   public SimpleTrajectoryPoint1D getJointTrajectoryPoint(int jointIndex, int trajectoryPointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex).getTrajectoryPoint(trajectoryPointIndex);
   }

   public OneDoFJointTrajectoryCommand getJointTrajectoryPointList(int jointIndex)
   {
      return jointTrajectoryInputs.get(jointIndex);
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
}
