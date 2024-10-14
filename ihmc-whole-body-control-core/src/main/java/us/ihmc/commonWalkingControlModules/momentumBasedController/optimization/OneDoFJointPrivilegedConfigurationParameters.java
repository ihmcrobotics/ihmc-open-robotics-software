package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.euclid.interfaces.Settable;

public class OneDoFJointPrivilegedConfigurationParameters implements Settable<OneDoFJointPrivilegedConfigurationParameters>
{
   private double privilegedConfiguration;
   private PrivilegedConfigurationOption privilegedConfigurationOption;
   private double weight;
   private double configurationGain;
   private double velocityGain;
   private double maxVelocity;
   private double maxAcceleration;

   public OneDoFJointPrivilegedConfigurationParameters()
   {
      clear();
   }

   public void clear()
   {
      privilegedConfiguration = Double.NaN;
      privilegedConfigurationOption = null;
      weight = Double.NaN;
      configurationGain = Double.NaN;
      velocityGain = Double.NaN;
      maxVelocity = Double.NaN;
      maxAcceleration = Double.NaN;
   }

   @Override
   public void set(OneDoFJointPrivilegedConfigurationParameters other)
   {
      privilegedConfiguration = other.privilegedConfiguration;
      privilegedConfigurationOption = other.privilegedConfigurationOption;
      weight = other.weight;
      configurationGain = other.configurationGain;
      velocityGain = other.velocityGain;
      maxVelocity = other.maxVelocity;
      maxAcceleration = other.maxAcceleration;
   }

   public void setPrivilegedConfiguration(double privilegedConfiguration)
   {
      this.privilegedConfiguration = privilegedConfiguration;
   }

   public void setPrivilegedConfigurationOption(PrivilegedConfigurationOption privilegedConfigurationOption)
   {
      this.privilegedConfigurationOption = privilegedConfigurationOption;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void setConfigurationGain(double configurationGain)
   {
      this.configurationGain = configurationGain;
   }

   public void setVelocityGain(double velocityGain)
   {
      this.velocityGain = velocityGain;
   }

   public void setMaxVelocity(double maxVelocity)
   {
      this.maxVelocity = maxVelocity;
   }

   public void setMaxAcceleration(double maxAcceleration)
   {
      this.maxAcceleration = maxAcceleration;
   }

   public boolean hasPrivilegedConfiguration()
   {
      return !Double.isNaN(privilegedConfiguration);
   }

   public boolean hasPrivilegedConfigurationOption()
   {
      return privilegedConfigurationOption != null;
   }

   public boolean hasWeight()
   {
      return !Double.isNaN(weight);
   }

   public boolean hasConfigurationGain()
   {
      return !Double.isNaN(configurationGain);
   }

   public boolean hasVelocityGain()
   {
      return !Double.isNaN(velocityGain);
   }

   public boolean hasMaxVelocity()
   {
      return !Double.isNaN(maxVelocity);
   }

   public boolean hasMaxAcceleration()
   {
      return !Double.isNaN(maxAcceleration);
   }

   public double getPrivilegedConfiguration()
   {
      return privilegedConfiguration;
   }

   public PrivilegedConfigurationOption getPrivilegedConfigurationOption()
   {
      return privilegedConfigurationOption;
   }

   public double getWeight()
   {
      return weight;
   }

   public double getConfigurationGain()
   {
      return configurationGain;
   }

   public double getVelocityGain()
   {
      return velocityGain;
   }

   public double getMaxVelocity()
   {
      return maxVelocity;
   }

   public double getMaxAcceleration()
   {
      return maxAcceleration;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof OneDoFJointPrivilegedConfigurationParameters)
      {
         OneDoFJointPrivilegedConfigurationParameters other = (OneDoFJointPrivilegedConfigurationParameters) object;

         if (Double.compare(privilegedConfiguration, other.privilegedConfiguration) != 0)
            return false;
         if (privilegedConfigurationOption != other.privilegedConfigurationOption)
            return false;
         if (Double.compare(weight, other.weight) != 0)
            return false;
         if (Double.compare(configurationGain, other.configurationGain) != 0)
            return false;
         if (Double.compare(velocityGain, other.velocityGain) != 0)
            return false;
         if (Double.compare(maxVelocity, other.maxVelocity) != 0)
            return false;
         if (Double.compare(maxAcceleration, other.maxAcceleration) != 0)
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ":";
      if (hasPrivilegedConfiguration())
         ret += " q_priv: " + privilegedConfiguration;
      if (hasPrivilegedConfigurationOption())
         ret += " q_priv option: " + privilegedConfigurationOption;
      if (hasWeight())
         ret += " weight: " + weight;
      if (hasConfigurationGain())
         ret += " configuration gain: " + configurationGain;
      if (hasVelocityGain())
         ret += " velocity gain: " + velocityGain;
      if (hasMaxVelocity())
         ret += " max velocity: " + maxVelocity;
      if (hasMaxAcceleration())
         ret += " max acceleration: " + maxAcceleration;
      return ret;
   }
}
