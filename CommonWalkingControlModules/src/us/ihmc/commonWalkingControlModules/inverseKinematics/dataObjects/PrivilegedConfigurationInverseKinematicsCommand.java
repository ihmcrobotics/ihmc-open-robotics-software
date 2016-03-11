package us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class PrivilegedConfigurationInverseKinematicsCommand extends InverseKinematicsCommand<PrivilegedConfigurationInverseKinematicsCommand>
{
   private final int initialCapacity = 40;
   private final List<String> jointNames = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   private final TDoubleArrayList privilegedOneDoFJointConfigurations = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList weights = new TDoubleArrayList(initialCapacity);

   private boolean enable = false;

   public enum PrivilegedConfigurationOption
   {
      AT_CURRENT, AT_MID_RANGE, AT_ZERO
   };

   private PrivilegedConfigurationOption option;
   private double defaultWeight = Double.NaN;

   public PrivilegedConfigurationInverseKinematicsCommand()
   {
      super(InverseKinematicsCommandType.PRIVILIEGED_CONFIGURATION);
   }

   public void clear()
   {
      enable = false;
      option = null;
      defaultWeight = Double.NaN;
      jointNames.clear();
      joints.clear();
      privilegedOneDoFJointConfigurations.reset();
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
      this.option = option;
   }

   public void addJointWithPrivilegedConfigurationOnly(OneDoFJoint joint, double privilegedConfiguration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(privilegedConfiguration);
      weights.add(Double.NaN);
   }

   public void addJointWithWeightOnly(OneDoFJoint joint, double weight)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(Double.NaN);
      weights.add(weight);
   }

   public void addJoint(OneDoFJoint joint, double privilegedConfiguration, double weight)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointConfigurations.add(privilegedConfiguration);
      weights.add(weight);
   }

   @Override
   public void set(PrivilegedConfigurationInverseKinematicsCommand other)
   {
      clear();
      enable = other.enable;
      option = other.option;
      defaultWeight = other.defaultWeight;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointNames.add(other.jointNames.get(i));
         privilegedOneDoFJointConfigurations.add(other.privilegedOneDoFJointConfigurations.get(i));
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

   public boolean hasNewPrivilegedConfigurationOption()
   {
      return option != null;
   }

   public PrivilegedConfigurationOption getPrivilegedConfigurationOption()
   {
      return option;
   }

   public boolean hasNewPrivilegedConfiguration(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointConfigurations.get(jointIndex));
   }

   public double getPrivilegedConfiguration(int jointIndex)
   {
      return privilegedOneDoFJointConfigurations.get(jointIndex);
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }
}
