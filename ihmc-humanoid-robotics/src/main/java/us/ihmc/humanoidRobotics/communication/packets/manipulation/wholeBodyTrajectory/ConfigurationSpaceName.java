package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

public enum ConfigurationSpaceName
{
   X, Y, Z, ROLL, PITCH, YAW;

   public double getDefaultExplorationLowerLimit()
   {
      switch (this)
      {
      case X:
      case Y:
      case Z:
         return -1.0;
      case ROLL:
      case PITCH:
      case YAW:
         return -Math.PI;
      default:
         throw new RuntimeException("Unexpected value: " + this);
      }
   }

   public double getDefaultExplorationUpperLimit()
   {
      switch (this)
      {
      case X:
      case Y:
      case Z:
         return 1.0;
      case ROLL:
      case PITCH:
      case YAW:
         return Math.PI;
      default:
         throw new RuntimeException("Unexpected value: " + this);
      }
   }
}