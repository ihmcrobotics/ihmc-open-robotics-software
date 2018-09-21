package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import gnu.trove.list.TByteList;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;

public enum ConfigurationSpaceName
{
   X, Y, Z, ROLL, PITCH, YAW, SO3;

   private static final boolean generateSE3ByQuaternion = true;

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

   public RigidBodyTransform getLocalRigidBodyTransform(double... configuration)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      switch (this)
      {
      case X:
         ret.appendTranslation(configuration[0], 0, 0);
         break;
      case Y:
         ret.appendTranslation(0, configuration[0], 0);
         break;
      case Z:
         ret.appendTranslation(0, 0, configuration[0]);
         break;
      case ROLL:
         ret.appendRollRotation(configuration[0]);
         break;
      case PITCH:
         ret.appendPitchRotation(configuration[0]);
         break;
      case YAW:
         ret.appendYawRotation(configuration[0]);
         break;
      case SO3:
         // uniform quat         
         if (generateSE3ByQuaternion)
         {
            Quaternion quat = new Quaternion();

            double s = configuration[0];
            double s1 = Math.sqrt(1 - s);
            double s2 = Math.sqrt(s);

            double theta1 = Math.PI * 2 * configuration[1];
            double theta2 = Math.PI * 2 * configuration[2];

            quat.set(Math.sin(theta1) * s1, Math.cos(theta1) * s1, Math.sin(theta2) * s2, Math.cos(theta2) * s2);
            quat.normalize();

            RotationMatrix rotationMatrix = new RotationMatrix(quat);
            ret.setRotation(rotationMatrix);
         }
         // uniform roll pitch yaw
         else
         {
            double theta1 = Math.PI * 2 * configuration[0];
            double theta2 = Math.acos(1 - 2 * configuration[1]) + Math.PI * 0.5;
            if (configuration[1] < 0.5)
               if (theta2 < Math.PI)
                  theta2 = theta2 + Math.PI;
               else
                  theta2 = theta2 - Math.PI;
            double theta3 = Math.PI * 2 * configuration[2] - Math.PI;

            ret.appendRollRotation(theta1);
            ret.appendPitchRotation(theta2);
            ret.appendYawRotation(theta3);
         }
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

   public static ConfigurationSpaceName[] fromBytes(TByteList enumListAsBytes)
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