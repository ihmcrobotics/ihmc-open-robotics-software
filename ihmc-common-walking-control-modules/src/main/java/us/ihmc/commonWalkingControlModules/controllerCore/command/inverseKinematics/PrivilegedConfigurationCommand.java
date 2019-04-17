package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class PrivilegedConfigurationCommand
      implements InverseKinematicsCommand<PrivilegedConfigurationCommand>, InverseDynamicsCommand<PrivilegedConfigurationCommand>
{
   /** Initial capacity of the internal memory. */
   private static final int initialCapacity = 40;

   /** different options for the desired privileged configurations. Made for ease of access. */
   public enum PrivilegedConfigurationOption
   {
      AT_CURRENT, AT_MID_RANGE, AT_ZERO
   }

   /** sets whether or not to utilize the privileged configuration calculator */
   private boolean enable = false;
   private final OneDoFJointPrivilegedConfigurationParameters defaultParameters = new OneDoFJointPrivilegedConfigurationParameters();
   /** internal memory to save the joints to be controlled. */
   private final List<OneDoFJointBasics> joints = new ArrayList<>(initialCapacity);
   private final RecyclingArrayList<OneDoFJointPrivilegedConfigurationParameters> jointSpecificParameters = new RecyclingArrayList<>(initialCapacity,
                                                                                                                                     OneDoFJointPrivilegedConfigurationParameters.class);

   /**
    * Creates an empty command.
    */
   public PrivilegedConfigurationCommand()
   {
      clear();
   }

   /**
    * Clears the data contained in this command.
    */
   public void clear()
   {
      enable = false;
      defaultParameters.clear();
      joints.clear();
      jointSpecificParameters.clear();
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
      defaultParameters.set(other.defaultParameters);

      for (int jointIndex = 0; jointIndex < other.getNumberOfJoints(); jointIndex++)
      {
         joints.add(other.joints.get(jointIndex));
         jointSpecificParameters.add().set(other.jointSpecificParameters.get(jointIndex));
      }
   }

   public void disable()
   {
      enable = false;
   }

   public void enable()
   {
      enable = true;
   }

   public void setDefaultParameters(OneDoFJointPrivilegedConfigurationParameters defaultParameters)
   {
      this.defaultParameters.set(defaultParameters);
   }

   public void setPrivilegedConfigurationOption(PrivilegedConfigurationOption option)
   {
      enable();
      defaultParameters.setPrivilegedConfigurationOption(option);
   }

   /**
    * Sets the new default weight for all privileged configurations to utilize.
    *
    * @param defaultWeight weight to use.
    */
   public void setDefaultWeight(double defaultWeight)
   {
      defaultParameters.setWeight(defaultWeight);
   }

   /**
    * Sets the new default configuration gain for all privileged configurations to utilize.
    *
    * @param defaultConfigurationGain position gain to use.
    */
   public void setDefaultConfigurationGain(double defaultConfigurationGain)
   {
      defaultParameters.setConfigurationGain(defaultConfigurationGain);
   }

   /**
    * Sets the new default velocity gain for all privileged configurations to utilize.
    *
    * @param defaultVelocityGain velocity gain to use.
    */
   public void setDefaultVelocityGain(double defaultVelocityGain)
   {
      defaultParameters.setVelocityGain(defaultVelocityGain);
   }

   public void setDefaultMaxVelocity(double defaultMaxVelocity)
   {
      defaultParameters.setMaxVelocity(defaultMaxVelocity);
   }

   public void setDefaultMaxAcceleration(double defaultMaxAcceleration)
   {
      defaultParameters.setMaxAcceleration(defaultMaxAcceleration);
   }

   public void addJoint(OneDoFJointBasics joint, OneDoFJointPrivilegedConfigurationParameters jointSpecificParameters)
   {
      enable();
      joints.add(joint);
      this.jointSpecificParameters.add().set(jointSpecificParameters);
   }

   /**
    * Adds a joint to set the privileged configuration for.
    *
    * @param joint the joint to set the configuration of.
    * @param privilegedConfiguration the desired privileged configuration for the joint to achieve.
    */
   public void addJoint(OneDoFJointBasics joint, double privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      OneDoFJointPrivilegedConfigurationParameters parameters = jointSpecificParameters.add();
      parameters.clear();
      parameters.setPrivilegedConfiguration(privilegedConfiguration);
   }

   /**
    * Adds a joint to set the privileged configuration option for.
    *
    * @param joint the joint to set the configuration of.
    * @param privilegedConfiguration the desired privileged configuration option for the joint to
    *           achieve.
    */
   public void addJoint(OneDoFJointBasics joint, PrivilegedConfigurationOption privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      OneDoFJointPrivilegedConfigurationParameters parameters = jointSpecificParameters.add();
      parameters.clear();
      parameters.setPrivilegedConfigurationOption(privilegedConfiguration);
   }

   public void setOneDoFJoint(int jointIndex, OneDoFJointPrivilegedConfigurationParameters jointSpecificParameters)
   {
      enable();
      this.jointSpecificParameters.get(jointIndex).set(jointSpecificParameters);
   }

   /**
    * Updates the desired privileged configuration for a joint already registered give its index.
    *
    * @param jointIndex index of the joint to set the configuration of.
    * @param privilegedConfiguration the desired privileged configuration for the joint to achieve.
    */
   public void setOneDoFJoint(int jointIndex, double privilegedConfiguration)
   {
      enable();
      OneDoFJointPrivilegedConfigurationParameters parameters = jointSpecificParameters.get(jointIndex);
      parameters.setPrivilegedConfiguration(privilegedConfiguration);
      parameters.setPrivilegedConfigurationOption(null);
   }

   /**
    * Updates the desired privileged configuration option for a joint already registered give its
    * index.
    *
    * @param jointIndex index of the joint to set the configuration opiton of.
    * @param privilegedConfiguration the desired privileged configuration option for the joint to
    *           achieve.
    */
   public void setOneDoFJoint(int jointIndex, PrivilegedConfigurationOption privilegedConfiguration)
   {
      enable();
      OneDoFJointPrivilegedConfigurationParameters parameters = jointSpecificParameters.get(jointIndex);
      parameters.setPrivilegedConfigurationOption(privilegedConfiguration);
      parameters.setPrivilegedConfiguration(Double.NaN);
   }

   public void setWeight(int jointIndex, double weight)
   {
      jointSpecificParameters.get(jointIndex).setWeight(weight);
   }

   public void setConfigurationGain(int jointIndex, double configurationGain)
   {
      jointSpecificParameters.get(jointIndex).setConfigurationGain(configurationGain);
   }

   public void setVelocityGain(int jointIndex, double velocityGain)
   {
      jointSpecificParameters.get(jointIndex).setVelocityGain(velocityGain);
   }

   public void setMaxVelocity(int jointIndex, double maxVelocity)
   {
      jointSpecificParameters.get(jointIndex).setMaxVelocity(maxVelocity);
   }

   public void setMaxAcceleration(int jointIndex, double maxAcceleration)
   {
      jointSpecificParameters.get(jointIndex).setMaxAcceleration(maxAcceleration);
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

   /**
    * Checks whether or not the privileged configuration is to be used.
    *
    * @return whether or not to use the privileged configuration.
    */
   public boolean isEnabled()
   {
      return enable;
   }

   public OneDoFJointPrivilegedConfigurationParameters getDefaultParameters()
   {
      return defaultParameters;
   }

   public OneDoFJointBasics getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   public OneDoFJointPrivilegedConfigurationParameters getJointSpecificParameters(int jointIndex)
   {
      return jointSpecificParameters.get(jointIndex);
   }

   public int getNumberOfJoints()
   {
      return jointSpecificParameters.size();
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.PRIVILEGED_CONFIGURATION;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof PrivilegedConfigurationCommand)
      {
         PrivilegedConfigurationCommand other = (PrivilegedConfigurationCommand) object;

         if (enable != other.enable)
            return false;
         if (!defaultParameters.equals(other.defaultParameters))
            return false;
         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (joints.get(jointIndex) != other.joints.get(jointIndex))
               return false;
         }
         if (!jointSpecificParameters.equals(other.jointSpecificParameters))
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
      String ret = getClass().getSimpleName() + ": enabled: " + enable + ", default parameters: " + defaultParameters.toString();
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
      {
         ret += "\nJoint: " + joints.get(jointIndex).getName() + ", " + jointSpecificParameters.get(jointIndex).toString();
      }
      return ret;
   }
}
