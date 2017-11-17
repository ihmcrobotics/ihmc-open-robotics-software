package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.euclid.transform.RigidBodyTransform;

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
   
   public RigidBodyTransform getLocalRigidBodyTransform(double configuration)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      switch (this)
      {
      case X:
         ret.appendTranslation(configuration, 0, 0);
         break;
      case Y:
         ret.appendTranslation(0, configuration, 0);
         break;
      case Z:
         ret.appendTranslation(0, 0, configuration);
         break;
      case ROLL:
         ret.appendRollRotation(configuration);
         break;
      case PITCH:
         ret.appendPitchRotation(configuration);
         break;
      case YAW:
         ret.appendYawRotation(configuration);
         break;
      }

      return ret;
   }
}