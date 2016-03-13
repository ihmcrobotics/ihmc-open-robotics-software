package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class PrivilegedConfigurationCommand implements InverseKinematicsCommand<PrivilegedConfigurationCommand>, InverseDynamicsCommand<PrivilegedConfigurationCommand>
{
   private final int initialCapacity = 40;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   private final TDoubleArrayList privilegedOneDoFJointConfigurations = new TDoubleArrayList(initialCapacity);
   private final Map<OneDoFJoint, PrivilegedConfigurationOption> privilegedOneDoFJointConfigurationOptions = new HashMap<>(initialCapacity);
   private final TDoubleArrayList weights = new TDoubleArrayList(initialCapacity);

   private boolean enable = false;

   public enum PrivilegedConfigurationOption
   {
      AT_CURRENT, AT_MID_RANGE, AT_ZERO
   };

   private PrivilegedConfigurationOption defaultOption;
   private double defaultWeight = Double.NaN;

   public PrivilegedConfigurationCommand()
   {
   }

   public void clear()
   {
      enable = false;
      defaultOption = null;
      defaultWeight = Double.NaN;
      jointNames.clear();
      joints.clear();
      privilegedOneDoFJointConfigurations.reset();
      privilegedOneDoFJointConfigurationOptions.clear();
      weights.reset();
   }

   public void disable()
   {
      enable = false;
   }

   public void enable()
   {
      enable = true;
   }

   public void setDefaultWeight(double defaultWeight)
   {
      this.defaultWeight = defaultWeight;
   }

   public void setPrivilegedConfigurationOption(PrivilegedConfigurationOption option)
   {
      enable();
      this.defaultOption = option;
   }

   public void addJointWithPrivilegedConfigurationOnly(OneDoFJoint joint, double privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(privilegedConfiguration);
      privilegedOneDoFJointConfigurationOptions.put(joint, null);
      weights.add(Double.NaN);
   }

   public void addJointWithPrivilegedConfigurationOnly(OneDoFJoint joint, PrivilegedConfigurationOption privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(Double.NaN);
      privilegedOneDoFJointConfigurationOptions.put(joint, privilegedConfiguration);
      weights.add(Double.NaN);
   }

   public void addJointWithWeightOnly(OneDoFJoint joint, double weight)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(Double.NaN);
      privilegedOneDoFJointConfigurationOptions.put(joint, null);
      weights.add(weight);
   }

   public void addJointsWithWeightOnly(OneDoFJoint[] joints, double weight)
   {
      for (int i = 0; i < joints.length; i++)
         addJointWithWeightOnly(joints[i], weight);
   }

   public void addJoint(OneDoFJoint joint, double privilegedConfiguration, double weight)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(privilegedConfiguration);
      privilegedOneDoFJointConfigurationOptions.put(joint, null);
      weights.add(weight);
   }

   public void addJoint(OneDoFJoint joint, PrivilegedConfigurationOption privilegedConfiguration, double weight)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(Double.NaN);
      privilegedOneDoFJointConfigurationOptions.put(joint, privilegedConfiguration);
      weights.add(weight);
   }

   @Override
   public void set(PrivilegedConfigurationCommand other)
   {
      clear();
      enable = other.enable;
      defaultOption = other.defaultOption;
      defaultWeight = other.defaultWeight;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = other.joints.get(i);
         joints.add(joint);
         jointNames.add(other.jointNames.get(i));
         privilegedOneDoFJointConfigurations.add(other.privilegedOneDoFJointConfigurations.get(i));
         privilegedOneDoFJointConfigurationOptions.put(joint, other.privilegedOneDoFJointConfigurationOptions.get(joint));
         weights.add(other.weights.get(i));
      }
   }

   public boolean isEnabled()
   {
      return enable;
   }

   public boolean hasNewDefaultWeight()
   {
      return !Double.isNaN(defaultWeight);
   }

   public double getDefaultWeight()
   {
      return defaultWeight;
   }

   public boolean hasNewWeight(int jointIndex)
   {
      return !Double.isNaN(weights.get(jointIndex));
   }

   public double getWeight(int jointIndex)
   {
      return weights.get(jointIndex);
   }

   public boolean hasNewPrivilegedConfigurationDefaultOption()
   {
      return defaultOption != null;
   }

   public PrivilegedConfigurationOption getPrivilegedConfigurationDefaultOption()
   {
      return defaultOption;
   }

   public boolean hasNewPrivilegedConfiguration(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointConfigurations.get(jointIndex));
   }

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
      return privilegedOneDoFJointConfigurationOptions.get(joints.get(jointIndex));
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
