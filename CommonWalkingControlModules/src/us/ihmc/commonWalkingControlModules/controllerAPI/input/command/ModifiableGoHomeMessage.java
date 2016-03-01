package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.EnumMap;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ModifiableGoHomeMessage
{
   private final SideDependentList<EnumMap<BodyPart, MutableBoolean>> sideDependentBodyPartRequestMap = SideDependentList.createListOfEnumMaps(BodyPart.class);
   private final EnumMap<BodyPart, MutableBoolean> otherBodyPartRequestMap = new EnumMap<>(BodyPart.class);

   public ModifiableGoHomeMessage()
   {
      for (BodyPart bodyPart : BodyPart.values)
      {
         if (bodyPart.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               sideDependentBodyPartRequestMap.get(robotSide).put(bodyPart, new MutableBoolean(false));
            }
         }
         else
         {
            otherBodyPartRequestMap.put(bodyPart, new MutableBoolean(false));
         }
      }
   }

   public void set(GoHomeMessage goHomeMessage)
   {
      BodyPart bodyPart = goHomeMessage.getBodyPart();
      if (bodyPart.isRobotSideNeeded())
      {
         RobotSide robotSide = goHomeMessage.getRobotSide();
         sideDependentBodyPartRequestMap.get(robotSide).get(bodyPart).setTrue();
      }
      else
      {
         otherBodyPartRequestMap.get(bodyPart).setTrue();
      }
   }

   public void set(ModifiableGoHomeMessage other)
   {
      for (BodyPart bodyPart : BodyPart.values)
      {
         if (bodyPart.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               sideDependentBodyPartRequestMap.get(robotSide).get(bodyPart).setValue(other.getRequest(robotSide, bodyPart));
            }
         }
         else
         {
            otherBodyPartRequestMap.get(bodyPart).setValue(other.getRequest(bodyPart));
         }
      }
   }
   
   private boolean getRequest(BodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
      return otherBodyPartRequestMap.get(bodyPart).booleanValue();
   }

   public boolean getRequest(RobotSide robotSide, BodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         return sideDependentBodyPartRequestMap.get(robotSide).get(bodyPart).booleanValue();
      else
         return getRequest(bodyPart);
   }
}
