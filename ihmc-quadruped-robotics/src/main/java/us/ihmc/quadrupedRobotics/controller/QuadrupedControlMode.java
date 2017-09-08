package us.ihmc.quadrupedRobotics.controller;

public enum QuadrupedControlMode
{
   POSITION,
   FORCE,
   POSITION_DEV,
   FORCE_DEV;

   public static QuadrupedControlMode fromCmdArgument(String argument)
   {
      if (argument.equals("--position"))
      {
         return POSITION;
      }
      else if (argument.equals("--force"))
      {
         return FORCE;
      }
      else if (argument.equals("--position-dev"))
      {
         return POSITION_DEV;
      }
      else if (argument.equals("--force-dev"))
      {
         return FORCE_DEV;
      }

      return null;
   }
}
