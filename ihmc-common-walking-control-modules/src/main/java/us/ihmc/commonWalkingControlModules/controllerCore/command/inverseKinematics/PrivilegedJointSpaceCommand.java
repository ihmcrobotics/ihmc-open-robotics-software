package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class PrivilegedJointSpaceCommand implements InverseKinematicsCommand<PrivilegedJointSpaceCommand>, InverseDynamicsCommand<PrivilegedJointSpaceCommand>
{
   /** Initial capacity of the internal memory. */
   private static final int initialCapacity = 40;

   /** sets whether or not to utilize the privileged acceleration calculator */
   private boolean enable = false;

   /** internal memory to save the joints to be controlled. */
   private final List<OneDoFJointBasics> joints = new ArrayList<>(initialCapacity);
   /** internal memory to save the desired joint space commands in */
   private final TDoubleArrayList privilegedOneDoFJointCommands = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList weights = new TDoubleArrayList(initialCapacity);

   /**
    * Creates an empty command.
    */
   public PrivilegedJointSpaceCommand()
   {
      clear();
   }

   /**
    * Clears the data contained in this command.
    */
   public void clear()
   {
      enable = false;
      joints.clear();
      privilegedOneDoFJointCommands.reset();
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

   public void setWeight(int jointIndex, double weight)
   {
      weights.set(jointIndex, weight);
   }

   /**
    * Adds a joint to set the privileged acceleration for.
    *
    * @param joint the joint to set the acceleration of.
    * @param privilegedAcceleration the desired privileged acceleration for the joint to achieve.
    */
   public void addJoint(OneDoFJointBasics joint, double privilegedAcceleration)
   {
      enable();
      joints.add(joint);
      privilegedOneDoFJointCommands.add(privilegedAcceleration);
      weights.add(Double.NaN);
   }

   /**
    * Updates the desired privileged acceleration for a joint already registered give its index.
    *
    * @param jointIndex index of the joint to set the acceleration of.
    * @param privilegedAcceleration the desired privileged acceleration for the joint to achieve.
    */
   public void setOneDoFJoint(int jointIndex, double privilegedAcceleration)
   {
      enable();
      privilegedOneDoFJointCommands.set(jointIndex, privilegedAcceleration);
   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not Modified.
    */
   @Override
   public void set(PrivilegedJointSpaceCommand other)
   {
      clear();
      enable = other.enable;

      for (int jointIndex = 0; jointIndex < other.getNumberOfJoints(); jointIndex++)
      {
         OneDoFJointBasics joint = other.joints.get(jointIndex);
         joints.add(joint);
         privilegedOneDoFJointCommands.add(other.privilegedOneDoFJointCommands.get(jointIndex));
         weights.add(other.weights.get(jointIndex));
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
      return weights.get(jointIndex);
   }

   /**
    * Returns whether or not there is a new command for all the privileged commands to use.
    *
    * @return if there is a new default command available.
    */
   public boolean hasNewPrivilegedCommand(int jointIndex)
   {
      return !Double.isNaN(privilegedOneDoFJointCommands.get(jointIndex));
   }

   /**
    * Returns the new acceleration.
    *
    * @return acceleration.
    */
   public double getPrivilegedCommand(int jointIndex)
   {
      return privilegedOneDoFJointCommands.get(jointIndex);
   }

   public int getNumberOfJoints()
   {
      return joints.size();
   }

   public OneDoFJointBasics getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.PRIVILEGED_JOINTSPACE_COMMAND;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof PrivilegedJointSpaceCommand)
      {
         PrivilegedJointSpaceCommand other = (PrivilegedJointSpaceCommand) object;

         if (isEnabled() != other.isEnabled())
            return false;
         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (joints.get(jointIndex) != other.joints.get(jointIndex))
               return false;
            if (Double.compare(weights.get(jointIndex), other.weights.get(jointIndex)) != 0)
               return false;
         }
         if (!privilegedOneDoFJointCommands.equals(other.privilegedOneDoFJointCommands))
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
      String ret = getClass().getSimpleName() + ": enabled: " + enable;
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
      {
         ret += "\nJoint: " + joints.get(jointIndex).getName();
         if (hasNewPrivilegedCommand(jointIndex))
            ret += ", command: " + getPrivilegedCommand(jointIndex);
         if (hasWeight(jointIndex))
            ret += ", weight: " + getWeight(jointIndex);
      }
      return ret;
   }
}
