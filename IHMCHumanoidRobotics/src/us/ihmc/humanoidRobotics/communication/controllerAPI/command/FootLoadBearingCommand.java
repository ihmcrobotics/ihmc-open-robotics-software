package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.CompilableCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootLoadBearingCommand implements CompilableCommand<FootLoadBearingCommand, FootLoadBearingMessage>
{
   private final SideDependentList<LoadBearingRequest> footRequests = new SideDependentList<>();

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
}
