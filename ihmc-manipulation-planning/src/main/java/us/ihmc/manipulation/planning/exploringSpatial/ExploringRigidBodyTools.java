package us.ihmc.manipulation.planning.exploringSpatial;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;

public class ExploringRigidBodyTools
{
   public static RigidBodyTransform getLocalRigidBodyTransform(ConfigurationSpaceName configurationSpaceName, double... configuration)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      switch (configurationSpaceName)
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
         break;
      }

      return ret;
   }
}
