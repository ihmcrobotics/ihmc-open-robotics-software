package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.AbstractDesiredAccelerationsMessage;

public abstract class DesiredAccelerationCommand<T extends DesiredAccelerationCommand<T, M>, M extends AbstractDesiredAccelerationsMessage<M>>
      implements Command<T, M>
{
   private final TDoubleArrayList desiredJointAccelerations = new TDoubleArrayList(10);
   
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;

   public DesiredAccelerationCommand()
   {
   }

   @Override
   public void clear()
   {
      desiredJointAccelerations.reset();
   }

   @Override
   public void set(M message)
   {
      desiredJointAccelerations.reset();
      for (int i = 0; i < message.getNumberOfJoints(); i++)
      {
         desiredJointAccelerations.add(message.getDesiredJointAcceleration(i));
      }
      executionDelayTime = message.getExecutionDelayTime();
   }

   @Override
   public void set(T other)
   {
      desiredJointAccelerations.reset();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
         desiredJointAccelerations.add(other.getDesiredJointAcceleration(i));
      executionDelayTime = other.getExecutionDelayTime();
   }

   public int getNumberOfJoints()
   {
      return desiredJointAccelerations.size();
   }

   public double getDesiredJointAcceleration(int jointIndex)
   {
      return desiredJointAccelerations.get(jointIndex);
   }

   public TDoubleArrayList getNeckDesiredJointAccelerations()
   {
      return desiredJointAccelerations;
   }

   @Override
   public boolean isCommandValid()
   {
      return !desiredJointAccelerations.isEmpty();
   }
   
   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }
}
