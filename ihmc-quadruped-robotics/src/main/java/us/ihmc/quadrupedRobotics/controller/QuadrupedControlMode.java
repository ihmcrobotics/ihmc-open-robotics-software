package us.ihmc.quadrupedRobotics.controller;

public enum QuadrupedControlMode
{
   POSITION,
   FORCE;

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

      return null;
   }
}
