package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.FootLoadBearingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootLoadBearingCommand implements Command<FootLoadBearingCommand, FootLoadBearingMessage>
{
   private long sequenceId;
   private final SideDependentList<LoadBearingRequest> footRequests = new SideDependentList<>();
   
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;

   public FootLoadBearingCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      footRequests.clear();
   }

   @Override
   public void setFromMessage(FootLoadBearingMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();
      LoadBearingRequest request = LoadBearingRequest.fromByte(message.getLoadBearingRequest());
      RobotSide robotSide = RobotSide.fromByte(message.getRobotSide());
      footRequests.put(robotSide, request);
      executionDelayTime = message.getExecutionDelayTime();
   }

   @Override
   public void set(FootLoadBearingCommand other)
   {
      clear();
      sequenceId = other.sequenceId;
      executionDelayTime = other.getExecutionDelayTime();
      for (RobotSide robotSide : RobotSide.values)
      {
         if (footRequests.get(robotSide) == null)
            footRequests.put(robotSide, other.getRequest(robotSide));
      }
   }


   public LoadBearingRequest getRequest(RobotSide robotSide)
   {
      return footRequests.get(robotSide);
   }

   @Override
   public Class<FootLoadBearingMessage> getMessageClass()
   {
      return FootLoadBearingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
