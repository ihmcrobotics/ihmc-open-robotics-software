package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

import java.util.EnumMap;

/**
 * @author twan
 *         Date: 5/6/13
 */
public enum HighLevelControllerName
{
   @RosEnumValueDocumentation(documentation = "do nothing state. the robot will start in this state, and report this state when falling and ramping down the controller. This state is intended for feedback only. Requesting this state is not supported and can cause the robot to shut down.")
   DO_NOTHING_BEHAVIOR,
   @RosEnumValueDocumentation(documentation = "Stand prep state.")
   STAND_PREP_STATE,
   @RosEnumValueDocumentation(documentation = "Stand ready state.")
   STAND_READY,
   @RosEnumValueDocumentation(documentation = "Freeze state.")
   FREEZE_STATE,
   @RosEnumValueDocumentation(documentation = "Stand transition state.")
   STAND_TRANSITION_STATE,
   @RosEnumValueDocumentation(documentation = "whole body force control employing IHMC walking, balance, and manipulation algorithms")
   WALKING,
   @RosEnumValueDocumentation(documentation = "Smooth transition state from walking to stand prep.")
   EXIT_WALKING,
   @RosEnumValueDocumentation(documentation = "The robot is peforming an automated diagnostic routine")
   DIAGNOSTICS,
   @RosEnumValueDocumentation(documentation = "Automated calibration routine depending on the robot. For Valkyrie: estimation of the joint torque offsets.")
   CALIBRATION,
   CUSTOM1,
   @RosEnumValueDocumentation(documentation = "State for recovering from a fall.")
   FALLING_STATE,
   @RosEnumValueDocumentation(documentation = "whole body force control employing IHMC push recovery algorithms")
   PUSH_RECOVERY,
   EXTERNAL_TRANSITION_STATE,
   EXTERNAL;


   public static final HighLevelControllerName[] values = values();

   private final static EnumMap<HighLevelControllerName, String> name = new EnumMap<>(HighLevelControllerName.class);

   static
   {
      for (HighLevelControllerName highLevelControllerName : values())
      {
         name.put(highLevelControllerName, highLevelControllerName.name());
      }
   }

   public static void setName(HighLevelControllerName state, String newName)
   {
      name.put(state, newName);
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   @Override
   public String toString()
   {
      return name.get(this);
   }

   public static HighLevelControllerName fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
