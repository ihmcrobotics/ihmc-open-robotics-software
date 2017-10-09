package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class OneDoFJointTrajectoryCommand extends SimpleTrajectoryPoint1DList implements Command<OneDoFJointTrajectoryCommand, OneDoFJointTrajectoryMessage>
{
   private long commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
   private double weight;
   
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;

   public OneDoFJointTrajectoryCommand()
   {
   }

   public OneDoFJointTrajectoryCommand(Random random)
   {
      super(random);
      commandId = random.nextInt(1000);
      executionDelayTime = random.nextDouble() * random.nextInt(1000);
      adjustedExecutionTime = random.nextDouble() * random.nextInt(1000);
      weight = random.nextDouble() * random.nextInt(1000);
   }

   @Override
   public void clear()
   {
      super.clear();
      commandId = Packet.VALID_MESSAGE_DEFAULT_ID;
      setWeight(Double.NaN);
      executionDelayTime = 0.0;
      adjustedExecutionTime = 0.0;
      
   }

   @Override
   public void set(OneDoFJointTrajectoryCommand other)
   {
      super.set(other);
      commandId = other.commandId;
      setWeight(other.getWeight());
   }

   @Override
   public void set(OneDoFJointTrajectoryMessage message)
   {
      message.getTrajectoryPoints(this);
      commandId = message.getUniqueId();
      setWeight(message.getWeight());
   }

   public void setCommandId(long commandId)
   {
      this.commandId = commandId;
   }

   public long getCommandId()
   {
      return commandId;
   }

   @Override
   public Class<OneDoFJointTrajectoryMessage> getMessageClass()
   {
      return OneDoFJointTrajectoryMessage.class;
   }

   public double getWeight()
   {
      return weight;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
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

   @Override
   public boolean isCommandValid()
   {
      boolean numberOfTrajectoryPointsIsPositive = getNumberOfTrajectoryPoints() > 0;
      boolean weightIsValid = !Double.isInfinite(getWeight());
      if (Double.isFinite(weight))
      {
         weightIsValid &= weight >= 0;
      }
      return numberOfTrajectoryPointsIsPositive && weightIsValid;
   }
   
   /**
    * returns the expected execution time of this command. The execution time will be computed when the controller 
    * receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }
   
   /**
    * tells the controller if this command supports delayed execution
    * (Spoiler alert: It does)
    * @return
    */
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }
}
