package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import java.util.EnumMap;

/**
 * @author twan
 *         Date: 5/6/13
 */
public enum HighLevelControllerName
{
   DO_NOTHING_BEHAVIOR,
   STAND_PREP_STATE,
   STAND_READY,
   FREEZE_STATE,
   STAND_TRANSITION_STATE,
   WALKING,
   EXIT_WALKING,
   DIAGNOSTICS,
   CALIBRATION,
   CUSTOM1,
   FALLING_STATE,
   QUICKSTER,
   EXTERNAL_TRANSITION_STATE,
   EXTERNAL,
   RL_CONTROL,
   EXIT_RL,
   RL_TRANSITION_STATE,
   @RosEnumValueDocumentation(documentation = "Ground prep state.")
   GROUND_PREP_STATE;

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
