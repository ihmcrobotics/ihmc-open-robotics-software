package us.ihmc.tools.inputDevices;

public enum JoystickDirection
{
   NORTH, NORTH_EAST, EAST, SOUTH_EAST, SOUTH, SOUTH_WEST, WEST, NORTH_WEST;

   public static JoystickDirection getFromJoystickPOV(float pov)
   {
      if ((pov < 0.0) || (pov > 360.0))
         return null;
      else if (((pov > 337.5) && (pov <= 360.0)) || ((pov >= 0.0) && (pov <= 22.5)))
         return NORTH;
      else if ((pov > 22.5) && (pov <= 67.5))
         return NORTH_EAST;
      else if ((pov > 67.5) && (pov <= 112.5))
         return EAST;
      else if ((pov > 112.5) && (pov <= 157.5))
         return SOUTH_EAST;
      else if ((pov > 157.5) && (pov <= 202.5))
         return SOUTH;
      else if ((pov > 202.5) && (pov <= 247.5))
         return SOUTH_WEST;
      else if ((pov > 247.5) && (pov <= 292.5))
         return WEST;
      else if ((pov > 292.5) && (pov <= 337.5))
         return NORTH_WEST;

      return null;
   }
}
