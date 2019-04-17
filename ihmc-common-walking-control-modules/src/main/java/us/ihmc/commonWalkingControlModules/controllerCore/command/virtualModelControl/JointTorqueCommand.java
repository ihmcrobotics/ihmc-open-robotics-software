package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.lists.DenseMatrixArrayList;

/**
 * {@link JointTorqueCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link JointTorqueCommand} is to notify the virtual model control module that
 * the given set of joints are to also include the joint torque during the next control tick.
 * </p>
 * <p>
 * It is usually the result of the {@link OneDoFJointFeedbackController}.
 * </p>
 * 
 * @author Robert Griffin
 *
 */
public class JointTorqueCommand implements VirtualModelControlCommand<JointTorqueCommand>
{
   /**
    * Initial capacity for the lists used in this command. It is to prevent memory allocation at the
    * beginning of the control.
    */
   private static final int initialCapacity = 15;
   /** The list of joints for which desired torques are assigned. */
   private final List<JointBasics> joints = new ArrayList<>(initialCapacity);
   /**
    * The list of the desired torques for each joint. The list follows the same ordering as the
    * {@link #joints} list. Each {@link DenseMatrix64F} in this list, is a N-by-1 vector where N is
    * equal to the number of degrees of freedom of the joint it is associated with.
    */
   private final DenseMatrixArrayList desiredTorques = new DenseMatrixArrayList(initialCapacity);

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public JointTorqueCommand()
   {
      clear();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(JointTorqueCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
      }
      desiredTorques.set(other.desiredTorques);
   }

   /**
    * Clears the internal memory. This action does not generate garbage, the data is simply 'marked' as
    * cleared but the memory will be recycled when setting up that command after calling this method.
    */
   public void clear()
   {
      joints.clear();
      desiredTorques.clear();
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index {@code i == this.getNumberOfJoints()}.
    * </p>
    * 
    * @param joint the joint to be controlled.
    * @param desiredTorque the joint torque to be achieved in the next control tick.
    */
   public void addJoint(OneDoFJointBasics joint, double desiredTorque)
   {
      joints.add(joint);
      DenseMatrix64F jointDesiredTorque = desiredTorques.add();
      jointDesiredTorque.reshape(1, 1);
      jointDesiredTorque.set(0, 0, desiredTorque);
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index {@code i == this.getNumberOfJoints()}.
    * </p>
    * 
    * @param joint the joint to be controlled.
    * @param desiredTorque the joint torque to be achieved in the next control tick. It is expected to
    *           be a N-by-1 vector with N equal to {@code joint.getDegreesOfFreedom()}. Not modified.
    * @throws RuntimeException if the {@code desiredTorque} is not a N-by-1 vector.
    */
   public void addJoint(JointBasics joint, DenseMatrix64F desiredTorque)
   {
      checkConsistency(joint, desiredTorque);
      joints.add(joint);
      desiredTorques.add().set(desiredTorque);
   }

   /**
    * Edits the desired torque for the {@code jointIndex}<sup>th</sup> joint in this command.
    * <p>
    * This method does not change the weight associated with the joint.
    * </p>
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @param desiredTorque the joint torque to be achieved in the next control tick.
    */
   public void setOneDoFJointDesiredTorque(int jointIndex, double desiredTorque)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredTorques.get(jointIndex).reshape(1, 1);
      desiredTorques.get(jointIndex).set(0, 0, desiredTorque);
   }

   /**
    * Edits the desired torque for the {@code jointIndex}<sup>th</sup> joint in this command.
    * <p>
    * This method does not change the weight associated with the joint.
    * </p>
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @param desiredTorque the joint torque to be achieved in the next control tick. It is expected to
    *           be a N-by-1 vector with N equal to {@code joint.getDegreesOfFreedom()}. Not modified.
    * @throws RuntimeException if the {@code desiredTorque} is not a N-by-1 vector.
    */
   public void setDesiredTorque(int jointIndex, DenseMatrix64F desiredTorque)
   {
      checkConsistency(joints.get(jointIndex), desiredTorque);
      desiredTorques.get(jointIndex).set(desiredTorque);
   }

   private void checkConsistency(JointBasics joint, DenseMatrix64F desiredTorque)
   {
      MathTools.checkEquals(joint.getDegreesOfFreedom(), desiredTorque.getNumRows());
   }

   /**
    * Gets the number of joints registered in this command.
    * 
    * @return the number of joints in this command.
    */
   public int getNumberOfJoints()
   {
      return joints.size();
   }

   /**
    * Gets the internal reference to this command's joint list.
    * 
    * @return the list of joints to be controlled.
    */
   public List<JointBasics> getJoints()
   {
      return joints;
   }

   /**
    * Gets the {@code jointIndex}<sup>th</th> joint in this command.
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @return one of the joints to be controlled.
    */
   public JointBasics getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   /**
    * Gets the desired torque associated with the {@code jointIndex}<sup>th</sup> joint of this
    * command.
    *
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @return the N-by-1 desired torque where N is the joint number of degrees of freedom.
    */
   public DenseMatrix64F getDesiredTorque(int jointIndex)
   {
      return desiredTorques.get(jointIndex);
   }

   /**
    * Gets the internal reference to this command's joint desired torque list.
    * 
    * @return the list of desired torques to be achieved by the joints.
    */
   public DenseMatrixArrayList getDesiredTorques()
   {
      return desiredTorques;
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#TASKSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINTSPACE;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof JointTorqueCommand)
      {
         JointTorqueCommand other = (JointTorqueCommand) object;

         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (joints.get(jointIndex) != other.joints.get(jointIndex))
               return false;
         }
         if (!desiredTorques.equals(other.desiredTorques))
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
      String ret = getClass().getSimpleName() + ": ";
      for (int i = 0; i < joints.size(); i++)
      {
         ret += joints.get(i).getName();
         if (i < joints.size() - 1)
            ret += ", ";
         else
            ret += ".";
      }
      return ret;
   }
}
