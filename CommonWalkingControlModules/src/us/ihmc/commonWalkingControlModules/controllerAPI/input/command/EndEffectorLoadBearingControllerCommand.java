package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.EnumMap;

import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class EndEffectorLoadBearingControllerCommand implements ControllerCommand<EndEffectorLoadBearingControllerCommand, EndEffectorLoadBearingMessage>
{
   private final SideDependentList<EnumMap<EndEffector, LoadBearingRequest>> sideDependentEndEffectorRequestMap = SideDependentList
         .createListOfEnumMaps(EndEffector.class);
   private final EnumMap<EndEffector, LoadBearingRequest> otherEndEffectorRequestMap = new EnumMap<>(EndEffector.class);

   public EndEffectorLoadBearingControllerCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      for (EndEffector endEffector : EndEffector.values)
      {
         if (endEffector.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               sideDependentEndEffectorRequestMap.get(robotSide).put(endEffector, null);
            }
         }
         else
         {
            otherEndEffectorRequestMap.put(endEffector, null);
         }
      }
   }

   @Override
   public void set(EndEffectorLoadBearingMessage message)
   {
      LoadBearingRequest request = message.getRequest();
      EndEffector endEffector = message.getEndEffector();

      if (endEffector.isRobotSideNeeded())
      {
         RobotSide robotSide = message.getRobotSide();
         sideDependentEndEffectorRequestMap.get(robotSide).put(endEffector, request);
      }
      else
      {
         otherEndEffectorRequestMap.put(endEffector, request);
      }
   }

   @Override
   public void set(EndEffectorLoadBearingControllerCommand other)
   {
      for (EndEffector endEffector : EndEffector.values)
      {
         if (endEffector.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               sideDependentEndEffectorRequestMap.get(robotSide).put(endEffector, other.getRequest(robotSide, endEffector));
            }
         }
         else
         {
            otherEndEffectorRequestMap.put(endEffector, other.getRequest(endEffector));
         }
      }
   }

   private LoadBearingRequest getRequest(EndEffector endEffector)
   {
      if (endEffector.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the endEffector: " + endEffector);
      return otherEndEffectorRequestMap.get(endEffector);
   }

   public LoadBearingRequest getRequest(RobotSide robotSide, EndEffector endEffector)
   {
      if (endEffector.isRobotSideNeeded())
         return sideDependentEndEffectorRequestMap.get(robotSide).get(endEffector);
      else
         return getRequest(endEffector);
   }

   @Override
   public Class<EndEffectorLoadBearingMessage> getMessageClass()
   {
      return EndEffectorLoadBearingMessage.class;
   }
}
