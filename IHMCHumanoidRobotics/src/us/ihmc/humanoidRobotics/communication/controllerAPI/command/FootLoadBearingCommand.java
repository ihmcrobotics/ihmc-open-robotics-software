package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.CompilableCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootLoadBearingCommand implements CompilableCommand<FootLoadBearingCommand, FootLoadBearingMessage>
{
   private final SideDependentList<LoadBearingRequest> footRequests = new SideDependentList<>();
   
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;

   public FootLoadBearingCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      footRequests.clear();
   }

   @Override
   public void set(FootLoadBearingMessage message)
   {
      clear();
      LoadBearingRequest request = message.getRequest();
      RobotSide robotSide = message.getRobotSide();
      footRequests.put(robotSide, request);
      executionDelayTime = message.executionDelayTime;
   }

   @Override
   public void set(FootLoadBearingCommand other)
   {
      clear();
      compile(other);
   }

   @Override
   public void compile(FootLoadBearingCommand other)
   {
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
}
