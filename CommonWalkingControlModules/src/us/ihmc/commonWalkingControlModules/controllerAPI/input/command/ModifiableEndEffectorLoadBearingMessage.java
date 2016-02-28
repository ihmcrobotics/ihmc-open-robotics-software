package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.EnumMap;

import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ModifiableEndEffectorLoadBearingMessage
{
   private final SideDependentList<EnumMap<EndEffector, LoadBearingRequest>> sideDependentEndEffectorRequestMap = SideDependentList.createListOfEnumMaps(EndEffector.class);
   private final EnumMap<EndEffector, LoadBearingRequest> otherEndEffectorRequestMap = new EnumMap<>(EndEffector.class);

   public ModifiableEndEffectorLoadBearingMessage()
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

   public void set(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage)
   {
      LoadBearingRequest request = endEffectorLoadBearingMessage.getRequest();
      EndEffector endEffector = endEffectorLoadBearingMessage.getEndEffector();

      if (endEffector.isRobotSideNeeded())
      {
         RobotSide robotSide = endEffectorLoadBearingMessage.getRobotSide();
         sideDependentEndEffectorRequestMap.get(robotSide).put(endEffector, request);
      }
      else
      {
         otherEndEffectorRequestMap.put(endEffector, request);
      }
   }
}
