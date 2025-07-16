package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.idl.IDLSequence;

public enum ConfigurationSpaceName
{
   X, Y, Z, ROLL, PITCH, YAW, SO3;

   public static final ConfigurationSpaceName[] values = values();

   public double getDefaultExplorationUpperLimit()
   {
      switch (this)
      {
      case X:
      case Y:
      case Z:
         return 0.1;
      case ROLL:
      case PITCH:
      case YAW:
         return 0.3 * Math.PI;
      case SO3:
         return 1.0;
      default:
         throw new RuntimeException("Unexpected value: " + this);
      }
   }

   public double getDefaultExplorationLowerLimit()
   {
      switch (this)
      {
      case X:
      case Y:
      case Z:
         return -0.1;
      case ROLL:
      case PITCH:
      case YAW:
         return -0.3 * Math.PI;
      case SO3:
         return 0.0;
      default:
         throw new RuntimeException("Unexpected value: " + this);
      }
   }

   public double getDefaultExplorationAmplitude()
   {
      switch (this)
      {
      case X:
      case Y:
      case Z:
         return 0.15;
      case ROLL:
      case PITCH:
      case YAW:
         return 0.3 * Math.PI;
      case SO3:
         return 1.0;
      default:
         throw new RuntimeException("Unexpected value: " + this);
      }
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

   public static ConfigurationSpaceName[] fromBytes(IDLSequence.Byte enumListAsBytes)
   {
      if (enumListAsBytes == null)
         return null;
      ConfigurationSpaceName[] enumArray = new ConfigurationSpaceName[enumListAsBytes.size()];
      for (int i = 0; i < enumListAsBytes.size(); i++)
         enumArray[i] = fromByte(enumListAsBytes.get(i));
      return enumArray;
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