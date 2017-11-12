package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class PrivilegedConfigurationCommand implements InverseKinematicsCommand<PrivilegedConfigurationCommand>, InverseDynamicsCommand<PrivilegedConfigurationCommand>
{
   /** Initial capacity of the internal memory. */
   private final int initialCapacity = 40;
   /**
    * Internal memory to save the names of the joints to be controlled. This is used when passing
    * the command between two modules using different instances of hte same physical robot.
    */
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   /** internal memory to save the joints to be controlled. */
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   /** internal memory to save the desired configurations in */
   private final RecyclingArrayList<MutableDouble> privilegedOneDoFJointConfigurations = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   /** internal memory to save the privileged configuration options in */
   private final TLongObjectHashMap<PrivilegedConfigurationOption> privilegedOneDoFJointConfigurationOptions;

   /** sets whether or not to utilize the privileged configuration calculator */
   private boolean enable = false;

   /** different options for the desired privileged configurations. Made for ease of access. */
   public enum PrivilegedConfigurationOption
   {
      AT_CURRENT, AT_MID_RANGE, AT_ZERO
   }

   private PrivilegedConfigurationOption option;
   private final RecyclingArrayList<MutableDouble> weights = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> configurationGains = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> velocityGains = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> maxVelocities = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   private final RecyclingArrayList<MutableDouble> maxAccelerations = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   private double defaultWeight = Double.NaN;
   private double defaultConfigurationGain = Double.NaN;
   private double defaultVelocityGain = Double.NaN;
   private double defaultMaxVelocity = Double.NaN;
   private double defaultMaxAcceleration = Double.NaN;

   /**
    * Creates an empty command.
    */
   public PrivilegedConfigurationCommand()
   {
      privilegedOneDoFJointConfigurationOptions = new TLongObjectHashMap<>(initialCapacity);

      clear();
   }

   /**
    * Clears the data contained in this command.
    */
   public void clear()
   {
      enable = false;
      option = null;
      jointNames.clear();
      joints.clear();
      privilegedOneDoFJointConfigurations.clear();
      privilegedOneDoFJointConfigurationOptions.clear();

      weights.clear();
      configurationGains.clear();
      velocityGains.clear();
      maxVelocities.clear();
      maxAccelerations.clear();
   }

   public void disable()
   {
      enable = false;
   }

   public void enable()
   {
      enable = true;
   }

   /**
    * Sets the new default weight for all privileged configurations to utilize.
    *
    * @param defaultWeight weight to use.
    */
   public void setDefaultWeight(double defaultWeight)
   {
      this.defaultWeight = defaultWeight;
   }

   /**
    * Sets the new default configuration gain for all privileged configurations to utilize.
    *
    * @param defaultConfigurationGain position gain to use.
    */
   public void setDefaultConfigurationGain(double defaultConfigurationGain)
   {
      this.defaultConfigurationGain = defaultConfigurationGain;
   }

   /**
    * Sets the new default velocity gain for all privileged configurations to utilize.
    *
    * @param defaultVelocityGain velocity gain to use.
    */
   public void setDefaultVelocityGain(double defaultVelocityGain)
   {
      this.defaultVelocityGain = defaultVelocityGain;
   }

   public void setDefaultMaxVelocity(double defaultMaxVelocity)
   {
      this.defaultMaxVelocity = defaultMaxVelocity;
   }

   public void setDefaultMaxAcceleration(double defaultMaxAcceleration)
   {
      this.defaultMaxAcceleration = defaultMaxAcceleration;
   }

   public void setWeight(int jointIndex, double weight)
   {
      weights.get(jointIndex).setValue(weight);
   }

   public void setConfigurationGain(int jointIndex, double configurationGain)
   {
      configurationGains.get(jointIndex).setValue(configurationGain);
   }

   public void setVelocityGain(int jointIndex, double velocityGain)
   {
      velocityGains.get(jointIndex).setValue(velocityGain);
   }

   public void setMaxVelocity(int jointIndex, double maxVelocity)
   {
      maxVelocities.get(jointIndex).setValue(maxVelocity);
   }

   public void setMaxAcceleration(int jointIndex, double maxAcceleration)
   {
      maxAccelerations.get(jointIndex).setValue(maxAcceleration);
   }

   public void setConfigurationGains(double configurationGain)
   {
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         setConfigurationGain(jointIndex, configurationGain);
   }

   public void setVelocityGains(double velocityGain)
   {
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         setVelocityGain(jointIndex, velocityGain);
   }

   public void setMaxVelocities(double maxVelocity)
   {
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         setMaxVelocity(jointIndex, maxVelocity);
   }

   public void setMaxAccelerations(double maxAcceleration)
   {
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         setMaxAcceleration(jointIndex, maxAcceleration);
   }

   public void setPrivilegedConfigurationOption(PrivilegedConfigurationOption option)
   {
      enable();
      this.option = option;
   }

   /**
    * Adds a joint to set the privileged configuration for.
    *
    * @param joint the joint to set the configuration of.
    * @param privilegedConfiguration the desired privileged configuration for the joint to achieve.
    */
   public void addJoint(OneDoFJoint joint, double privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add().setValue(privilegedConfiguration);
      privilegedOneDoFJointConfigurationOptions.put(joint.getNameBasedHashCode(), null);

      weights.add().setValue(Double.NaN);
      configurationGains.add().setValue(Double.NaN);
      velocityGains.add().setValue(Double.NaN);
      maxVelocities.add().setValue(Double.NaN);
      maxAccelerations.add().setValue(Double.NaN);
   }

   /**
    * Adds a joint to set the privileged configuration option for.
    *
    * @param joint the joint to set the configuration of.
    * @param privilegedConfiguration the desired privileged configuration option for the joint to achieve.
    */
   public void addJoint(OneDoFJoint joint, PrivilegedConfigurationOption privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add().setValue(Double.NaN);
      privilegedOneDoFJointConfigurationOptions.put(joint.getNameBasedHashCode(), privilegedConfiguration);

      weights.add().setValue(Double.NaN);
      configurationGains.add().setValue(Double.NaN);
      velocityGains.add().setValue(Double.NaN);
      maxVelocities.add().setValue(Double.NaN);
      maxAccelerations.add().setValue(Double.NaN);
   }

   /**
    * Updates the desired privileged configuration for a joint already registered give its index.
    *
    * @param jointIndex index of the joint to set the configuration of.
    * @param privilegedConfiguration the desired privileged configuration for the joint to achieve.
    */
   public void setOneDoFJoint(int jointIndex, double privilegedConfiguration)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      enable();
      privilegedOneDoFJointConfigurations.get(jointIndex).setValue(privilegedConfiguration);
      privilegedOneDoFJointConfigurationOptions.put(joints.get(jointIndex).getNameBasedHashCode(), null);
   }

   /**
    * Updates the desired privileged configuration option for a joint already registered give its index.
    *
    * @param jointIndex index of the joint to set the configuration opiton of.
    * @param privilegedConfiguration the desired privileged configuration option for the joint to achieve.
    */
   public void setOneDoFJoint(int jointIndex, PrivilegedConfigurationOption privilegedConfiguration)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      enable();
      privilegedOneDoFJointConfigurations.get(jointIndex).setValue(Double.NaN);
      privilegedOneDoFJointConfigurationOptions.put(joints.get(jointIndex).getNameBasedHashCode(), privilegedConfiguration);
   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not Modified.
    */
   @Override
   public void set(PrivilegedConfigurationCommand other)
   {
      clear();
      enable = other.enable;
      option = other.option;

      defaultWeight = other.defaultWeight;
      defaultConfigurationGain = other.defaultConfigurationGain;
      defaultVelocityGain = other.defaultVelocityGain;
      defaultMaxVelocity = other.defaultMaxVelocity;
      defaultMaxAcceleration = other.defaultMaxAcceleration;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = other.joints.get(i);
         joints.add(joint);
         jointNames.add(other.jointNames.get(i));
         privilegedOneDoFJointConfigurations.add().setValue(other.privilegedOneDoFJointConfigurations.get(i));
         privilegedOneDoFJointConfigurationOptions.put(joint.getNameBasedHashCode(), other.privilegedOneDoFJointConfigurationOptions.get(joint.getNameBasedHashCode()));

         weights.add().setValue(other.weights.get(i));
         configurationGains.add().setValue(other.configurationGains.get(i));
         velocityGains.add().setValue(other.velocityGains.get(i));
         maxVelocities.add().setValue(other.maxVelocities.get(i));
         maxAccelerations.add().setValue(other.maxAccelerations.get(i));
      }
   }

   /**
    * Checks whether or not the privileged configuration is to be used.
    *
    * @return whether or not to use the privileged configuration.
    */
   public boolean isEnabled()
   {
      return enable;
   }

   /**
    * Returns whether or not there is a specific weight for this privileged configuration command.
    *
    * @return if there is a weight available.
    */
   public boolean hasWeight(int jointIndex)
   {
      return !Double.isNaN(getWeight(jointIndex));
   }

   /**
    * Returns the specific weight for this privileged configuration.
    *
    * @return Weight.
    */
   public double getWeight(int jointIndex)
   {
      return weights.get(jointIndex).doubleValue();
   }

   /**
    * Returns whether or not there is a specific configuration gain for this privileged configuration command.
    *
    * @return if there is a configuration gain available.
    */
   public boolean hasConfigurationGain(int jointIndex)
   {
      return !Double.isNaN(getConfigurationGain(jointIndex));
   }

   /**
    * Returns the specific configuration gain for this privileged configuration.
    *
    * @return configuration gain.
    */
   public double getConfigurationGain(int jointIndex)
   {
      return configurationGains.get(jointIndex).doubleValue();
   }

   /**
    * Returns whether or not there is a specific velocity gain for this privileged configuration command.
    *
    * @return if there is a velocity gain available.
    */
   public boolean hasVelocityGain(int jointIndex)
   {
      return !Double.isNaN(getVelocityGain(jointIndex));
   }

   /**
    * Returns the specific velocity gain for this privileged configuration.
    *
    * @return velocity gain.
    */
   public double getVelocityGain(int jointIndex)
   {
      return velocityGains.get(jointIndex).doubleValue();
   }

   /**
    * Returns whether or not there is a specific max velocity for this privileged configuration command.
    *
    * @return if there is a max velocity available.
    */
   public boolean hasMaxVelocity(int jointIndex)
   {
      return !Double.isNaN(getMaxVelocity(jointIndex));
   }

   /**
    * Returns the specific max velocity for this privileged configuration.
    *
    * @return max velocity.
    */
   public double getMaxVelocity(int jointIndex)
   {
      return maxVelocities.get(jointIndex).doubleValue();
   }

   /**
    * Returns whether or not there is a specific max acceleration for this privileged configuration command.
    *
    * @return if there is a max acceleration available.
    */
   public boolean hasMaxAcceleration(int jointIndex)
   {
      return !Double.isNaN(getMaxAcceleration(jointIndex));
   }

   /**
    * Returns the specific max acceleration for this privileged configuration.
    *
    * @return max acceleration.
    */
   public double getMaxAcceleration(int jointIndex)
   {
      return maxAccelerations.get(jointIndex).doubleValue();
   }

   /**
    * Returns whether or not there is a new default weight for all the privileged configuration to use.
    *
    * @return if there is a new default weight available.
    */
   public boolean hasNewDefaultWeight()
   {
      return !Double.isNaN(defaultWeight);
   }

   /**
    * Returns the new default weight.
    *
    * @return default weight.
    */
   public double getDefaultWeight()
   {
      return defaultWeight;
   }

   /**
    * Returns whether or not there is a new default configuration gain for all the privileged configuration to use.
    *
    * @return if there is a new default configuration gain available.
    */
   public boolean hasNewDefaultConfigurationGain()
   {
      return !Double.isNaN(defaultConfigurationGain);
   }

   /**
    * Returns the new default configuration gain.
    *
    * @return default configuration gain.
    */
   public double getDefaultConfigurationGain()
   {
      return defaultConfigurationGain;
   }

   /**
    * Returns whether or not there is a new default velocity gain for all the privileged configuration to use.
    *
    * @return if there is a new default velocity gain available.
    */
   public boolean hasNewDefaultVelocityGain()
   {
      return !Double.isNaN(defaultVelocityGain);
   }

   /**
    * Returns the new default velocity gain.
    *
    * @return default velocity gain.
    */
   public double getDefaultVelocityGain()
   {
      return defaultVelocityGain;
   }

   /**
    * Returns whether or not there is a new default max velocity for all the privileged configuration to use.
    *
    * @return if there is a new default max velocity available.
    */
   public boolean hasNewDefaultMaxVelocity()
   {
      return !Double.isNaN(defaultMaxVelocity);
   }

   /**
    * Returns the new default max velocity.
    *
    * @return default max velocity.
    */
   public double getDefaultMaxVelocity()
   {
      return defaultMaxVelocity;
   }

   /**
    * Returns whether or not there is a new default max acceleration for all the privileged configuration to use.
    *
    * @return if there is a new default max acceleration available.
    */
   public boolean hasNewDefaultMaxAcceleration()
   {
      return !Double.isNaN(defaultMaxAcceleration);
   }

   /**
    * Returns the new default max acceleration.
    *
    * @return default max acceleration.
    */
   public double getDefaultMaxAcceleration()
   {
      return defaultMaxAcceleration;
   }

   /**
    * Returns whether or not there is a new default configuration option for all the privileged configuration to use.
    *
    * @return if there is a new default configuration option available.
    */
   public boolean hasNewPrivilegedConfigurationDefaultOption()
   {
      return option != null;
   }

   /**
    * Returns the new default configuration option.
    *
    * @return default configuration option.
    */
   public PrivilegedConfigurationOption getPrivilegedConfigurationDefaultOption()
   {
      return option;
   }

   /**
    * Returns whether or not there is a new default configuration for all the privileged configuration to use.
    *
    * @return if there is a new default configuration available.
    */
   public boolean hasNewPrivilegedConfiguration(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointConfigurations.get(jointIndex).doubleValue());
   }

   /**
    * Returns the new default configuration.
    *
    * @return default configuration.
    */
   public double getPrivilegedConfiguration(int jointIndex)
   {
      return privilegedOneDoFJointConfigurations.get(jointIndex).doubleValue();
   }

   public boolean hasNewPrivilegedConfigurationOption(int jointIndex)
   {
      return getPrivilegedConfigurationOption(jointIndex) != null;
   }

   public PrivilegedConfigurationOption getPrivilegedConfigurationOption(int jointIndex)
   {
      return privilegedOneDoFJointConfigurationOptions.get(joints.get(jointIndex).getNameBasedHashCode());
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public OneDoFJoint getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.PRIVILEGED_CONFIGURATION;
   }
}
