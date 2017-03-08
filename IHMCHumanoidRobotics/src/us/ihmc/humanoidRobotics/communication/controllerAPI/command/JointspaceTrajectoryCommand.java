package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.packets.AbstractJointspaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public abstract class JointspaceTrajectoryCommand<T extends JointspaceTrajectoryCommand<T, M>, M extends AbstractJointspaceTrajectoryMessage<M>> extends QueueableCommand<T, M>
{
   private final RecyclingArrayList<OneDoFJointTrajectoryCommand> jointTrajectoryInputs = new RecyclingArrayList<>(10, OneDoFJointTrajectoryCommand.class);

   public JointspaceTrajectoryCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      clearQueuableCommandVariables();
      jointTrajectoryInputs.clear();
   }

   @Override
   public void set(T other)
   {
      setQueueqableCommandVariables(other);
      set(other.getTrajectoryPointLists());
   }

   @Override
   public void set(M message)
   {
      setQueueqableCommandVariables(message);
      set(message.getTrajectoryPointLists());
   }

   private void set(RecyclingArrayList<? extends SimpleTrajectoryPoint1DList> trajectoryPointListArray)
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
         SimpleTrajectoryPoint1DList simpleTrajectoryPoint1DList = jointTrajectoryInputs.add();
         OneDoFJointTrajectoryMessage oneJointTrajectoryMessage = trajectoryPointListArray[i];
         oneJointTrajectoryMessage.getTrajectoryPoints(simpleTrajectoryPoint1DList);
      }
   }

   private void set(int jointIndex, SimpleTrajectoryPoint1DList otherTrajectoryPointList)
   {
      SimpleTrajectoryPoint1DList thisJointTrajectoryPointList = jointTrajectoryInputs.getAndGrowIfNeeded(jointIndex);
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

   public SimpleTrajectoryPoint1DList getJointTrajectoryPointList(int jointIndex)
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
}
