package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

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
   private final TDoubleArrayList privilegedOneDoFJointConfigurations;
   /** internal memory to save the privileged configuration options in */
   private final TLongObjectHashMap<PrivilegedConfigurationOption> privilegedOneDoFJointConfigurationOptions;

   /** sets whether or not to utilize the privileged configuration calculator */
   private boolean enable = false;

   /** different options for the desired privileged configurations. Made for ease of access. */
   public enum PrivilegedConfigurationOption
   {
      AT_CURRENT, AT_MID_RANGE, AT_ZERO
   }

   private PrivilegedConfigurationOption defaultOption;
   private double weight = Double.NaN;
   private double configurationGain = Double.NaN;
   private double velocityGain = Double.NaN;
   private double maxVelocity = Double.NaN;
   private double maxAcceleration = Double.NaN;

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
      privilegedOneDoFJointConfigurations = new TDoubleArrayList(initialCapacity);
      privilegedOneDoFJointConfigurationOptions = new TLongObjectHashMap<PrivilegedConfigurationOption>(initialCapacity);
      
      clear();
   }

   /**
    * Clears the data contained in this command.
    */
   public void clear()
   {
      enable = false;
      defaultOption = null;
      weight = Double.NaN;
      configurationGain = Double.NaN;
      velocityGain = Double.NaN;
      maxVelocity = Double.NaN;
      maxAcceleration = Double.NaN;
      jointNames.clear();
      joints.clear();
      privilegedOneDoFJointConfigurations.reset();
      privilegedOneDoFJointConfigurationOptions.clear();
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

   public void setPrivilegedConfigurationOption(PrivilegedConfigurationOption option)
   {
      enable();
      this.defaultOption = option;
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
      privilegedOneDoFJointConfigurations.add(privilegedConfiguration);
      privilegedOneDoFJointConfigurationOptions.put(joint.getNameBasedHashCode(), null);
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
      privilegedOneDoFJointConfigurations.add(Double.NaN);
      privilegedOneDoFJointConfigurationOptions.put(joint.getNameBasedHashCode(), privilegedConfiguration);
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
      privilegedOneDoFJointConfigurations.set(jointIndex, privilegedConfiguration);
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
      privilegedOneDoFJointConfigurations.set(jointIndex, Double.NaN);
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
      defaultOption = other.defaultOption;

      weight = other.weight;
      configurationGain = other.configurationGain;
      velocityGain = other.velocityGain;
      maxVelocity = other.maxVelocity;
      maxAcceleration = other.maxAcceleration;

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
         privilegedOneDoFJointConfigurations.add(other.privilegedOneDoFJointConfigurations.get(i));
         privilegedOneDoFJointConfigurationOptions.put(joint.getNameBasedHashCode(), other.privilegedOneDoFJointConfigurationOptions.get(joint.getNameBasedHashCode()));
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
   public boolean hasWeight()
   {
      return !Double.isNaN(weight);
   }

   /**
    * Returns the specific weight for this privileged configuration.
    *
    * @return Weight.
    */
   public double getWeight()
   {
      return weight;
   }

   /**
    * Returns whether or not there is a specific configuration gain for this privileged configuration command.
    *
    * @return if there is a configuration gain available.
    */
   public boolean hasConfigurationGain()
   {
      return !Double.isNaN(configurationGain);
   }

   /**
    * Returns the specific configuration gain for this privileged configuration.
    *
    * @return configuration gain.
    */
   public double getConfigurationGain()
   {
      return configurationGain;
   }

   /**
    * Returns whether or not there is a specific velocity gain for this privileged configuration command.
    *
    * @return if there is a velocity gain available.
    */
   public boolean hasVelocityGain()
   {
      return !Double.isNaN(velocityGain);
   }

   /**
    * Returns the specific velocity gain for this privileged configuration.
    *
    * @return velocity gain.
    */
   public double getVelocityGain()
   {
      return velocityGain;
   }

   /**
    * Returns whether or not there is a specific max velocity for this privileged configuration command.
    *
    * @return if there is a max velocity available.
    */
   public boolean hasMaxVelocity()
   {
      return !Double.isNaN(maxVelocity);
   }

   /**
    * Returns the specific max velocity for this privileged configuration.
    *
    * @return max velocity.
    */
   public double getMaxVelocity()
   {
      return maxVelocity;
   }

   /**
    * Returns whether or not there is a specific max acceleration for this privileged configuration command.
    *
    * @return if there is a max acceleration available.
    */
   public boolean hasMaxAcceleration()
   {
      return !Double.isNaN(maxAcceleration);
   }

   /**
    * Returns the specific max acceleration for this privileged configuration.
    *
    * @return max acceleration.
    */
   public double getMaxAcceleration()
   {
      return maxAcceleration;
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
      return defaultOption != null;
   }

   /**
    * Returns the new default configuration option.
    *
    * @return default configuration option.
    */
   public PrivilegedConfigurationOption getPrivilegedConfigurationDefaultOption()
   {
      return defaultOption;
   }

   /**
    * Returns whether or not there is a new default configuration for all the privileged configuration to use.
    *
    * @return if there is a new default configuration available.
    */
   public boolean hasNewPrivilegedConfiguration(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointConfigurations.get(jointIndex));
   }

   /**
    * Returns the new default configuration.
    *
    * @return default configuration.
    */
   public double getPrivilegedConfiguration(int jointIndex)
   {
      return privilegedOneDoFJointConfigurations.get(jointIndex);
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
