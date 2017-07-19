package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.manipulation.planning.trajectory.ConfigurationBuildOrder.ConfigurationSpaceName;

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
   
   private RigidBodyTransform getLocalRigidBodyTransform(ConfigurationSpaceName configurationName)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      
      switch(configurationName)
      {
      case Translation_X:
         ret.appendTranslation(getTranslationX(), 0, 0);
         break;
      case Translation_Y:
         ret.appendTranslation(0, getTranslationY(), 0);
         break;
      case Translation_Z:
         ret.appendTranslation(0, 0, getTranslationZ());
         break;
      case Rotation_Roll:
         ret.appendRollRotation(getRotationRoll());
         break;
      case Rotation_Pitch:
         ret.appendPitchRotation(getRotationPitch());
         break;
      case Rotation_Yaw:
         ret.appendYawRotation(getRotationYaw());
         break;
      }
      
      return ret;
   }
   
   public RigidBodyTransform createRigidBodyTransform(ConfigurationBuildOrder configurationBuildOrder)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      
      for(int i=0;i<6;i++)
      {
         ret.multiply(getLocalRigidBodyTransform(configurationBuildOrder.getConfigurationSpaceName(i)));
      }
      
      return ret;
   }
}
