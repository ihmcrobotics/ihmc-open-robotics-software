package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

/*
 * @link ConfigurationSpace has same physical meaning with transform matrix such as RighdBodyTransform, Pose3D, etc.
 * @link Configurationspace can be converted into other transform matrix according to ConfigurationBuildOrder @link ConfigurationBuildOrder.
 */
public class ConfigurationSpace
{
   private double translationX;
   private double translationY;
   private double translationZ;

   /*
    * the all rotation is intrinsic.(appending on moving frame)
    */
   private double rotationRoll;
   private double rotationPitch;
   private double rotationYaw;

   public ConfigurationSpace()
   {
      this.translationX = 0;
      this.translationY = 0;
      this.translationZ = 0;
      this.rotationRoll = 0;
      this.rotationPitch = 0;
      this.rotationYaw = 0;
   }

   public ConfigurationSpace(double x, double y, double z, double roll, double pitch, double yaw)
   {
      this.translationX = x;
      this.translationY = y;
      this.translationZ = z;
      this.rotationRoll = roll;
      this.rotationPitch = pitch;
      this.rotationYaw = yaw;
   }

   public void setTranslation(double x, double y, double z)
   {
      translationX = x;
      translationY = y;
      translationZ = z;
   }

   public void setTranslation(Tuple3DReadOnly translation)
   {
      translationX = translation.getX();
      translationY = translation.getY();
      translationZ = translation.getZ();
   }

   public void setRotation(double roll, double pitch, double yaw)
   {
      this.rotationRoll = roll;
      this.rotationPitch = pitch;
      this.rotationYaw = yaw;
   }

   public double getTranslationX()
   {
      return translationX;
   }

   public double getTranslationY()
   {
      return translationY;
   }

   public double getTranslationZ()
   {
      return translationZ;
   }

   public double getRotationRoll()
   {
      return rotationRoll;
   }

   public double getRotationPitch()
   {
      return rotationPitch;
   }

   public double getRotationYaw()
   {
      return rotationYaw;
   }

   public double getConfigurationSpace(ConfigurationSpaceName configurationName)
   {
      double ret = 0;

      switch (configurationName)
      {
      case X:
         ret = getTranslationX();
         break;
      case Y:
         ret = getTranslationY();
         break;
      case Z:
         ret = getTranslationZ();
         break;
      case ROLL:
         ret = getRotationRoll();
         break;
      case PITCH:
         ret = getRotationPitch();
         break;
      case YAW:
         ret = getRotationYaw();
         break;
      }

      return ret;
   }

   private RigidBodyTransform getLocalRigidBodyTransform(ConfigurationSpaceName configurationName)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      switch (configurationName)
      {
      case X:
         ret.appendTranslation(getTranslationX(), 0, 0);
         break;
      case Y:
         ret.appendTranslation(0, getTranslationY(), 0);
         break;
      case Z:
         ret.appendTranslation(0, 0, getTranslationZ());
         break;
      case ROLL:
         ret.appendRollRotation(getRotationRoll());
         break;
      case PITCH:
         ret.appendPitchRotation(getRotationPitch());
         break;
      case YAW:
         ret.appendYawRotation(getRotationYaw());
         break;
      }

      return ret;
   }

   public RigidBodyTransform createRigidBodyTransform(ConfigurationBuildOrder configurationBuildOrder)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      for (int i = 0; i < 6; i++)
      {
         ret.multiply(getLocalRigidBodyTransform(configurationBuildOrder.getConfigurationSpaceName(i)));
      }

      return ret;
   }

   public ConfigurationSpace overrideConfigurationSpaceCopy(SelectionMatrix6D overrideSelectionMatrix, ConfigurationSpace overrideConfigurationSpace)
   {
      double translationX = 0;
      double translationY = 0;
      double translationZ = 0;

      double rotationRoll = 0;
      double rotationPitch = 0;
      double rotationYaw = 0;
      
      for (ConfigurationSpaceName configurationName : ConfigurationSpaceName.values())
      {
         switch (configurationName)
         {
         case X:
            if (overrideSelectionMatrix.isLinearXSelected())
               translationX = overrideConfigurationSpace.getTranslationX();
            else
               translationX = this.getTranslationX();
            break;
         case Y:
            if (overrideSelectionMatrix.isLinearYSelected())
               translationY = overrideConfigurationSpace.getTranslationY();
            else
               translationY = this.getTranslationY();
            break;
         case Z:
            if (overrideSelectionMatrix.isLinearZSelected())
               translationZ = overrideConfigurationSpace.getTranslationZ();
            else
               translationZ = this.getTranslationZ();
            break;
         case ROLL:
            if (overrideSelectionMatrix.isAngularXSelected())
               rotationRoll = overrideConfigurationSpace.getRotationRoll();
            else
               rotationRoll = this.getRotationRoll();
            break;
         case PITCH:
            if (overrideSelectionMatrix.isAngularYSelected())
               rotationPitch = overrideConfigurationSpace.getRotationPitch();
            else
               rotationPitch = this.getRotationPitch();
            break;
         case YAW:
            if (overrideSelectionMatrix.isAngularZSelected())
               rotationYaw = overrideConfigurationSpace.getRotationYaw();
            else
               rotationYaw = this.getRotationYaw();
            break;
         }
      }
      
      return new ConfigurationSpace(translationX, translationY, translationZ, rotationRoll, rotationPitch, rotationYaw);
   }
}