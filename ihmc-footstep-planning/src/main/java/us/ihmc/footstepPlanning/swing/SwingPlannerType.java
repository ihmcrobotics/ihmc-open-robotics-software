package us.ihmc.footstepPlanning.swing;

import us.ihmc.robotics.trajectories.TrajectoryType;

/**
 * Desired plan type, corresponds to {@link TrajectoryType}. If not listed, that plan type currently isn't supported.
 */
public enum SwingPlannerType
{
   NONE,
   POSITION,
   PROPORTION;

   public static SwingPlannerType fromByte(byte index)
   {
      if (index == -1)
      {
         return null;
      }
      else
      {
         return values()[index];
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }
}
