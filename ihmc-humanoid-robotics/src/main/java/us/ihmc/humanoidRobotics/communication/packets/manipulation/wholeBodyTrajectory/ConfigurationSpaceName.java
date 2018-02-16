package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.euclid.transform.RigidBodyTransform;

public enum ConfigurationSpaceName
{
   X, Y, Z, ROLL, PITCH, YAW;

   public static final ConfigurationSpaceName[] values = values();

   public double getDefaultExplorationLowerLimit()
   {
      return -getDefaultExplorationAmplitude();
   }

   public double getDefaultExplorationUpperLimit()
   {
      return getDefaultExplorationAmplitude();
   }

   public double getDefaultExplorationAmplitude()
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
         return 0.25 * Math.PI;
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

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static ConfigurationSpaceName fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }

   public static byte[] toBytes(ConfigurationSpaceName[] enumArray)
   {
      if (enumArray == null)
         return null;
      byte[] byteArray = new byte[enumArray.length];
      for (int i = 0; i < enumArray.length; i++)
         byteArray[i] = enumArray[i].toByte();
      return byteArray;
   }
   
   public static ConfigurationSpaceName[] fromBytes(byte[] enumArrayAsBytes)
   {
      if (enumArrayAsBytes == null)
         return null;
      ConfigurationSpaceName[] enumArray = new ConfigurationSpaceName[enumArrayAsBytes.length];
      for (int i = 0; i < enumArrayAsBytes.length; i++)
         enumArray[i] = fromByte(enumArrayAsBytes[i]);
      return enumArray;
   }
}