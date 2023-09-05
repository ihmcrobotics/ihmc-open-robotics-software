package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import static us.ihmc.robotics.weightMatrices.SolverWeightLevels.HARD_CONSTRAINT;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
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
public class JointTorqueCommand implements VirtualModelControlCommand<JointTorqueCommand>, InverseDynamicsCommand<JointTorqueCommand>
{
   /**
    * Initial capacity for the lists used in this command. It is to prevent memory allocation at the
    * beginning of the control.
    */
   private static final int initialCapacity = 15;
   private int commandId;
   /** The list of joints for which desired torques are assigned. */
   private final List<JointBasics> joints = new ArrayList<>(initialCapacity);
   /**
    * The list of the desired torques for each joint. The list follows the same ordering as the
    * {@link #joints} list. Each {@link DMatrixRMaj} in this list, is a N-by-1 vector where N is
    * equal to the number of degrees of freedom of the joint it is associated with.
    */
   private final DenseMatrixArrayList desiredTorques = new DenseMatrixArrayList(initialCapacity);
   /**
    * The list of weights to use for each joint. The list follows the same ordering as the
    * {@link #joints} list. A higher weight means higher priority of the joint task.
    */
   private final TDoubleArrayList weights = new TDoubleArrayList(initialCapacity);
   /**
    * Specifies the constraint type to use. If it is an objective, this command will be entered as a cost objective
    * into the QP. If it is equality, it will be entered as an equality constraint. If it is an inequality,
    * it will be used to set either the upper or lower acceleration bound in the QP.
    */
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

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

      commandId = other.commandId;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         weights.add(other.weights.get(i));
      }
      desiredTorques.set(other.desiredTorques);
      constraintType = other.getConstraintType();
   }

   /**
    * Clears the internal memory. This action does not generate garbage, the data is simply 'marked' as
    * cleared but the memory will be recycled when setting up that command after calling this method.
    */
   public void clear()
   {
      commandId = 0;
      joints.clear();
      desiredTorques.clear();
      weights.reset();
   }

   /**
    * Sets the constraint type to be used in the QP for this command.
    */
   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index {@code i == this.getNumberOfJoints()}.
    * Note that it is registered as a hard constraint. It is highly recommended to set the weight
    * afterwards or simply use {@link #addJoint(OneDoFJointBasics, double, double)} instead.
    * </p>
    * 
    * @param joint the joint to be controlled.
    * @param desiredTorque the joint torque to be achieved in the next control tick.
    */
   public void addJoint(OneDoFJointBasics joint, double desiredTorque)
   {
      addJoint(joint, desiredTorque, HARD_CONSTRAINT);
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
   public void addJoint(OneDoFJointBasics joint, double desiredTorque, double weight)
   {
      joints.add(joint);
      weights.add(weight);
      DMatrixRMaj jointDesiredTorque = desiredTorques.add();
      jointDesiredTorque.reshape(1, 1);
      jointDesiredTorque.set(0, 0, desiredTorque);
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index {@code i == this.getNumberOfJoints()}.
    * Note that it is registered as a hard constraint. It is highly recommended to set the weight
    * afterwards.
    * </p>
    *
    * @param joint the joint to be controlled.
    * @param desiredTorque the joint torque to be achieved in the next control tick. It is expected to
    *           be a N-by-1 vector with N equal to {@code joint.getDegreesOfFreedom()}. Not modified.
    * @throws RuntimeException if the {@code desiredTorque} is not a N-by-1 vector.
    */
   public void addJoint(JointBasics joint, DMatrixRMaj desiredTorque)
   {
      addJoint(joint, desiredTorque, HARD_CONSTRAINT);
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
    * @param weight positive value that denotes the priority of the joint task.
    * @throws RuntimeException if the {@code desiredTorque} is not a N-by-1 vector.
    */
   public void addJoint(JointBasics joint, DMatrixRMaj desiredTorque, double weight)
   {
      checkConsistency(joint, desiredTorque);
      joints.add(joint);
      weights.add(weight);
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
   public void setDesiredTorque(int jointIndex, DMatrixRMaj desiredTorque)
   {
      checkConsistency(joints.get(jointIndex), desiredTorque);
      desiredTorques.get(jointIndex).set(desiredTorque);
   }

   /**
    * Edits the weight value of the {@code jointIndex}<sup>th</sup> joint of this command.
    *
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @param weight positive value that denotes the priority of the joint task.
    */
   public void setWeight(int jointIndex, double weight)
   {
      weights.set(jointIndex, weight);
   }

   /**
    * Sets the weight of each joint in this command to the given one.
    *
    * @param weight positive value that denotes the priority of the command.
    */
   public void setWeight(double weight)
   {
      for (int jointIdx = 0; jointIdx < joints.size(); jointIdx++)
         weights.set(jointIdx, weight);
   }

   private static void checkConsistency(JointBasics joint, DMatrixRMaj desiredTorque)
   {
      MathTools.checkEquals(joint.getDegreesOfFreedom(), desiredTorque.getNumRows());
   }

   /**
    * Finds if this command is to be considered as a hard constraint during the optimization.
    * <p>
    * Note that the joints in this command should all be setup either as hard or soft constraints.
    * </p>
    *
    * @return {@code true} if this command should be considered as a hard constraint, {@code false} is
    *         it should be part of the optimization objective.
    * @throws RuntimeException if not all the joints in this command are setup as hard constraints.
    */
   public boolean isHardConstraint()
   {
      if (getNumberOfJoints() == 0)
         return true;
      if (constraintType == ConstraintType.EQUALITY)
         return true;

      boolean isHardConstraint = getWeight(0) == HARD_CONSTRAINT;
      if (getNumberOfJoints() == 1)
         return isHardConstraint;

      // If there are multiple joints, make sure they are consistent.
      for (int jointIdx = 1; jointIdx < joints.size(); jointIdx++)
      {
         boolean isJointHardConstraint = getWeight(jointIdx) == HARD_CONSTRAINT;
         if (isJointHardConstraint != isHardConstraint)
            throw new RuntimeException("Inconsistent weights in " + getClass().getSimpleName() + ": some joint acceleration "
                                       + "desireds have weights, others are hard constraints. This is not supported in a single message.");
      }

      return isHardConstraint;
   }

   /**
    * Gets the constraint type to be used in the QP for this command.
    */
   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   /**
    * Gets the weight associated with the {@code jointIndex}<sup>th</sup> joint of this command.
    *
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @return the weight value.
    */
   public double getWeight(int jointIndex)
   {
      return weights.get(jointIndex);
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
   public DMatrixRMaj getDesiredTorque(int jointIndex)
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
    * @return {@link ControllerCoreCommandType#JOINTSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINTSPACE;
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof JointTorqueCommand other)
      {
         if (commandId != other.commandId)
            return false;
         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         if (constraintType != other.getConstraintType())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (joints.get(jointIndex) != other.joints.get(jointIndex))
               return false;
         }
         if (!desiredTorques.equals(other.desiredTorques))
            return false;
         if (!weights.equals(other.weights))
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
