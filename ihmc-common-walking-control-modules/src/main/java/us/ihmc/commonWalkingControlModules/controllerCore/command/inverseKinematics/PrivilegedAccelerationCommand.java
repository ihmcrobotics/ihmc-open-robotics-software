package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.MathTools;
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
   /** internal memory to save the desired accelerations in */
   private final RecyclingArrayList<MutableDouble> privilegedOneDoFJointAccelerations = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   /** sets whether or not to utilize the privileged acceleration calculator */
   private boolean enable = false;

   private final RecyclingArrayList<MutableDouble> weights = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

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

   public void setWeight(int jointIndex, double weight)
   {
      weights.get(jointIndex).setValue(weight);
   }

   /**
    * Adds a joint to set the privileged acceleration for.
    *
    * @param joint the joint to set the acceleration of.
    * @param privilegedAcceleration the desired privileged acceleration for the joint to achieve.
    */
   public void addJoint(OneDoFJoint joint, double privilegedAcceleration)
   {
      enable();
      joints.add(joint);
      jointNames.add(joint.getName());
      privilegedOneDoFJointAccelerations.add().setValue(privilegedAcceleration);

      weights.add().setValue(Double.NaN);
   }


   /**
    * Updates the desired privileged acceleration for a joint already registered give its index.
    *
    * @param jointIndex index of the joint to set the acceleration of.
    * @param privilegedAcceleration the desired privileged acceleration for the joint to achieve.
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
    * Checks whether or not the privileged acceleration is to be used.
    *
    * @return whether or not to use the privileged acceleration.
    */
   public boolean isEnabled()
   {
      return enable;
   }

   /**
    * Returns whether or not there is a specific weight for this privileged acceleration command.
    *
    * @return if there is a weight available.
    */
   public boolean hasWeight(int jointIndex)
   {
      return !Double.isNaN(getWeight(jointIndex));
   }

   /**
    * Returns the specific weight for this privileged acceleration.
    *
    * @return Weight.
    */
   public double getWeight(int jointIndex)
   {
      return weights.get(jointIndex).doubleValue();
   }

   /**
    * Returns whether or not there is a new acceleration for all the privileged acceleration to use.
    *
    * @return if there is a new default acceleration available.
    */
   public boolean hasNewPrivilegedAcceleration(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointAccelerations.get(jointIndex).doubleValue());
   }

   /**
    * Returns the new acceleration.
    *
    * @return acceleration.
    */
   public double getPrivilegedAcceleration(int jointIndex)
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
