package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.ArrayList;
import java.util.List;

public class PrivilegedAccelerationCommand implements InverseKinematicsCommand<PrivilegedAccelerationCommand>, InverseDynamicsCommand<PrivilegedAccelerationCommand>
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
   private final RecyclingArrayList<MutableDouble> privilegedOneDoFJointAccelerations = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   /** sets whether or not to utilize the privileged configuration calculator */
   private boolean enable = false;

   private final RecyclingArrayList<MutableDouble> weights = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   private double defaultWeight = Double.NaN;

   /**
    * Creates an empty command.
    */
   public PrivilegedAccelerationCommand()
   {
      clear();
   }

   /**
    * Clears the data contained in this command.
    */
   public void clear()
   {
      enable = false;
      jointNames.clear();
      joints.clear();
      privilegedOneDoFJointAccelerations.clear();

      weights.clear();
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

   public void setWeight(int jointIndex, double weight)
   {
      weights.get(jointIndex).setValue(weight);
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
      privilegedOneDoFJointAccelerations.add().setValue(privilegedConfiguration);

      weights.add().setValue(Double.NaN);
   }


   /**
    * Updates the desired privileged acceleration for a joint already registered give its index.
    *
    * @param jointIndex index of the joint to set the acceleration of.
    * @param privilegedConfiguration the desired privileged acceleration for the joint to achieve.
    */
   public void setOneDoFJoint(int jointIndex, double privilegedAcceleration)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      enable();
      privilegedOneDoFJointAccelerations.get(jointIndex).setValue(privilegedAcceleration);
   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not Modified.
    */
   @Override
   public void set(PrivilegedAccelerationCommand other)
   {
      clear();
      enable = other.enable;

      defaultWeight = other.defaultWeight;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         OneDoFJoint joint = other.joints.get(i);
         joints.add(joint);
         jointNames.add(other.jointNames.get(i));
         privilegedOneDoFJointAccelerations.add().setValue(other.privilegedOneDoFJointAccelerations.get(i));

         weights.add().setValue(other.weights.get(i));
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
    * Returns whether or not there is a new default configuration for all the privileged configuration to use.
    *
    * @return if there is a new default configuration available.
    */
   public boolean hasNewPrivilegedConfiguration(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointAccelerations.get(jointIndex).doubleValue());
   }

   /**
    * Returns the new default configuration.
    *
    * @return default configuration.
    */
   public double getPrivilegedConfiguration(int jointIndex)
   {
      return privilegedOneDoFJointAccelerations.get(jointIndex).doubleValue();
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
      return ControllerCoreCommandType.PRIVILEGED_ACCELERATION;
   }
}
