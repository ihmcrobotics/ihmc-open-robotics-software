package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static us.ihmc.robotics.weightMatrices.SolverWeightLevels.HARD_CONSTRAINT;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.jointspace.OneDoFJointFeedbackController;
import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;

/**
 * {@link JointspaceVelocityCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link JointspaceVelocityCommand} is to notify the inverse kinematics
 * optimization module that the given set of joints are to track a set of desired velocities during
 * the next control tick.
 * </p>
 * <p>
 * It is usually the result of the {@link OneDoFJointFeedbackController}.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class JointspaceVelocityCommand implements InverseKinematicsCommand<JointspaceVelocityCommand>
{
   /**
    * Initial capacity for the lists used in this command. It is to prevent memory allocation at the
    * beginning of the control.
    */
   private static final int initialCapacity = 15;
   /** The list of joints for which desired velocities are assigned. */
   private final List<JointBasics> joints = new ArrayList<>(initialCapacity);
   /**
    * The list of the desired velocities for each joint. The list follows the same ordering as the
    * {@link #joints} list. Each {@link DenseMatrix64F} in this list, is a N-by-1 vector where N is
    * equal to the number of degrees of freedom of the joint it is associated with.
    */
   private final DenseMatrixArrayList desiredVelocities = new DenseMatrixArrayList(initialCapacity);

   /**
    * The list of weights to use for each joint. The list follows the same ordering as the
    * {@link #joints} list. A higher weight means higher priority of the joint task.
    */
   private final TDoubleArrayList weights = new TDoubleArrayList(initialCapacity);

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public JointspaceVelocityCommand()
   {
      clear();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(JointspaceVelocityCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         weights.add(other.getWeight(i));
      }
      desiredVelocities.set(other.desiredVelocities);
   }

   /**
    * Clears the internal memory. This action does not generate garbage, the data is simply 'marked' as
    * cleared but the memory will be recycled when setting up that command after calling this method.
    */
   public void clear()
   {
      joints.clear();
      desiredVelocities.clear();
      weights.reset();
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
    * @param desiredVelocity the joint velocity to be achieved in the next control tick.
    */
   public void addJoint(OneDoFJointBasics joint, double desiredVelocity)
   {
      addJoint(joint, desiredVelocity, HARD_CONSTRAINT);
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index {@code i == this.getNumberOfJoints()}.
    * </p>
    * 
    * @param joint the joint to be controlled.
    * @param desiredVelocity the joint velocity to be achieved in the next control tick.
    * @param weight positive value that denotes the priority of the joint task.
    */
   public void addJoint(OneDoFJointBasics joint, double desiredVelocity, double weight)
   {
      joints.add(joint);
      weights.add(weight);
      DenseMatrix64F jointDesiredVelocity = desiredVelocities.add();
      jointDesiredVelocity.reshape(1, 1);
      jointDesiredVelocity.set(0, 0, desiredVelocity);
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
    * @param desiredVelocity the joint velocity to be achieved in the next control tick. It is expected
    *           to be a N-by-1 vector with N equal to {@code joint.getDegreesOfFreedom()}. Not
    *           modified.
    * @throws RuntimeException if the {@code desiredAcceleration} is not a N-by-1 vector.
    */
   public void addJoint(JointBasics joint, DenseMatrix64F desiredVelocity)
   {
      checkConsistency(joint, desiredVelocity);
      joints.add(joint);
      desiredVelocities.add().set(desiredVelocity);
   }

   /**
    * Adds a joint to be controlled to this command.
    * <p>
    * The joint is added at the last position, i.e. at the index {@code i == this.getNumberOfJoints()}.
    * </p>
    * 
    * @param joint the joint to be controlled.
    * @param desiredVelocity the joint velocity to be achieved in the next control tick. It is expected
    *           to be a N-by-1 vector with N equal to {@code joint.getDegreesOfFreedom()}. Not
    *           modified.
    * @param weight positive value that denotes the priority of the joint task.
    * @throws RuntimeException if the {@code desiredAcceleration} is not a N-by-1 vector.
    */
   public void addJoint(JointBasics joint, DenseMatrix64F desiredVelocity, double weight)
   {
      checkConsistency(joint, desiredVelocity);
      joints.add(joint);
      weights.add(weight);
      desiredVelocities.add().set(desiredVelocity);
   }

   /**
    * Edits the desired velocity for the {@code jointIndex}<sup>th</sup> joint in this command.
    * <p>
    * This method does not change the weight associated with the joint.
    * </p>
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @param desiredVelocity the joint velocity to be achieved in the next control tick.
    */
   public void setOneDoFJointDesiredVelocity(int jointIndex, double desiredVelocity)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredVelocities.get(jointIndex).reshape(1, 1);
      desiredVelocities.get(jointIndex).set(0, 0, desiredVelocity);
   }

   /**
    * Edits the desired velocity for the {@code jointIndex}<sup>th</sup> joint in this command.
    * <p>
    * This method does not change the weight associated with the joint.
    * </p>
    * 
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @param desiredVelocity the joint velocity to be achieved in the next control tick. It is expected
    *           to be a N-by-1 vector with N equal to {@code joint.getDegreesOfFreedom()}. Not
    *           modified.
    * @throws RuntimeException if the {@code desiredVelocity} is not a N-by-1 vector.
    */
   public void setDesiredVelocity(int jointIndex, DenseMatrix64F desiredVelocity)
   {
      checkConsistency(joints.get(jointIndex), desiredVelocity);
      desiredVelocities.get(jointIndex).set(desiredVelocity);
   }

   /**
    * Sets all the weights to {@link SolverWeightLevels#HARD_CONSTRAINT} such that this command will be
    * treated as a hard constraint.
    * <p>
    * This is usually undesired as with improper commands setup as hard constraints the optimization
    * problem can simply be impossible to solve.
    * </p>
    */
   public void setAsHardConstraint()
   {
      for (int jointIdx = 0; jointIdx < joints.size(); jointIdx++)
         setWeight(jointIdx, HARD_CONSTRAINT);
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

   private void checkConsistency(JointBasics joint, DenseMatrix64F desiredVelocity)
   {
      MathTools.checkEquals(joint.getDegreesOfFreedom(), desiredVelocity.getNumRows());
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

      boolean isHardConstraint = getWeight(0) == HARD_CONSTRAINT;
      if (getNumberOfJoints() == 1)
         return isHardConstraint;

      // If there are multiple joints, make sure they are consistent.
      for (int jointIdx = 1; jointIdx < joints.size(); jointIdx++)
      {
         boolean isJointHardConstraint = getWeight(jointIdx) == HARD_CONSTRAINT;
         if (isJointHardConstraint != isHardConstraint)
            throw new RuntimeException("Inconsistent weights in " + getClass().getSimpleName() + ": some joint velocity "
                  + "desireds have weights, others are hard constraints. This is not supported in a single message.");
      }

      return isHardConstraint;
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
    * Gets the desired velocity associated with the {@code jointIndex}<sup>th</sup> joint of this
    * command.
    *
    * @param jointIndex the index of the joint &in; [0, {@code getNumberOfJoints()}[.
    * @return the N-by-1 desired velocity where N is the joint number of degrees of freedom.
    */
   public DenseMatrix64F getDesiredVelocity(int jointIndex)
   {
      return desiredVelocities.get(jointIndex);
   }

   /**
    * Gets the internal reference to this command's joint desired velocity list.
    * 
    * @return the list of desired velocitys to be achieved by the joints.
    */
   public DenseMatrixArrayList getDesiredVelocities()
   {
      return desiredVelocities;
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
      else if (object instanceof JointspaceVelocityCommand)
      {
         JointspaceVelocityCommand other = (JointspaceVelocityCommand) object;

         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (joints.get(jointIndex) != other.joints.get(jointIndex))
               return false;
         }
         if (!desiredVelocities.equals(other.desiredVelocities))
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
