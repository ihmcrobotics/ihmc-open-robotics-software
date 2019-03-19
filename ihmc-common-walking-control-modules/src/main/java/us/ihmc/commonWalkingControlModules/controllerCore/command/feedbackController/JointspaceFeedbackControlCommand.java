package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;

/**
 * A {@code JointspaceFeedbackControlCommand} can be used to request the
 * {@link WholeBodyFeedbackController} to run PD controllers on a list of {@link OneDoFJointBasics}s
 * to reach given desired positions and desired velocities.
 * <p>
 * The PD controllers used also handle a feed-forward acceleration for improved performance.
 * </p>
 * <p>
 * In addition to the desireds, the gains to be used in the PD controllers have to be provided
 * alongside with the weight used in the QP optimization problem, see for instance
 * {@link WholeBodyInverseDynamicsSolver}.
 * </p>
 * <p>
 * Every control tick, a {@code JointspaceFeedbackControlCommand} has to be sent to the controller
 * core allowing the higher-level controller to continuously update the desireds, gains, and weight
 * to use.
 * </p>
 *
 *
 * @author Sylvain Bertrand
 */
public class JointspaceFeedbackControlCommand implements FeedbackControlCommand<JointspaceFeedbackControlCommand>
{
   /** Initial capacity of the internal memory. */
   private final int initialCapacity = 15;
   /** Internal memory to save the joints to be controlled */
   private final List<OneDoFJointBasics> joints = new ArrayList<>(initialCapacity);

   /** The desired positions for each controlled joint. */
   private final TDoubleArrayList desiredPositions = new TDoubleArrayList(initialCapacity);
   /** The desired velocities for each controlled joint. */
   private final TDoubleArrayList desiredVelocities = new TDoubleArrayList(initialCapacity);
   /**
    * The feed-forward to be used for each controlled joint. Useful to improve tracking performance.
    */
   private final TDoubleArrayList feedForwardAccelerations = new TDoubleArrayList(initialCapacity);

   /** Gains to used by the PD controllers for the next control tick. */
   private final RecyclingArrayList<PDGains> gains = new RecyclingArrayList<>(initialCapacity, PDGains.class);
   /** Weight used in the QP optimization describing how 'important' achieving this command is. */
   private final TDoubleArrayList weightsForSolver = new TDoubleArrayList(initialCapacity);

   /**
    * Creates an empty command.
    */
   public JointspaceFeedbackControlCommand()
   {
      clear();
   }

   /**
    * Clears the data contained in this command.
    */
   public void clear()
   {
      joints.clear();
      desiredPositions.clear();
      desiredVelocities.clear();
      feedForwardAccelerations.clear();
      weightsForSolver.clear();
      gains.clear();
   }

   /**
    * Updates the gains to be used for the next control tick. This will update the gains for a specific
    * joint.
    *
    * @param jointIndex the index of the joint that should use the given gains.
    * @param gains the new set of gains to be used. The data is copied into a local object. Not
    *           modified.
    */
   public void setGains(int jointIndex, PDGainsReadOnly gains)
   {
      this.gains.get(jointIndex).set(gains);
   }

   /**
    * Updates the gains to be used for the next control tick. This will update the gains for all joints
    * in this command.
    *
    * @param gains the new set of gains to be used. The data is copied into a local object. Not
    *           modified.
    */
   public void setGains(PDGainsReadOnly gains)
   {
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         this.gains.get(jointIndex).set(gains);
   }

   /**
    * Updates the weight to be used for the next control tick. This will update the weight for a
    * specific joint.
    * <p>
    * This relates to how 'important' is this command compared to other commands submitted.
    * </p>
    *
    * @param jointIndex the index of the joint that should use the given weight.
    * @param weight the new joint weight for the QP optimization.
    */
   public void setWeightForSolver(int jointIndex, double weight)
   {
      weightsForSolver.set(jointIndex, weight);
   }

   /**
    * Updates the weights to be used for the next control tick. This will update the weights for all
    * joints in this command.
    * <p>
    * This relates to how 'important' is this command compared to other commands submitted.
    * </p>
    *
    * @param weight the new weight for the QP optimization.
    */
   public void setWeightForSolver(double weight)
   {
      for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         weightsForSolver.set(jointIndex, weight);
   }

   /**
    * Adds a joint to be controlled in the next control tick.
    *
    * @param joint the joint to be controlled.
    * @param desiredPosition the joint's desired position.
    * @param desiredVelocity the joint's desired velocity.
    * @param feedForwardAcceleration the feed-forward acceleration for the joint, used to improve
    *           tracking performance.
    */
   public void addJoint(OneDoFJointBasics joint, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      joints.add(joint);
      desiredPositions.add(desiredPosition);
      desiredVelocities.add(desiredVelocity);
      feedForwardAccelerations.add(feedForwardAcceleration);
      weightsForSolver.add(Double.POSITIVE_INFINITY);
      gains.add().set(0.0, 0.0, 0.0, 0.0);
   }

   /**
    * Adds a joint to be controlled in the next control tick.
    *
    * @param joint the joint to be controlled.
    * @param desiredPosition the joint's desired position.
    * @param desiredVelocity the joint's desired velocity.
    * @param feedForwardAcceleration the feed-forward acceleration for the joint, used to improve
    *           tracking performance.
    */
   public void addJoint(OneDoFJointBasics joint, double desiredPosition, double desiredVelocity, double feedForwardAcceleration, PDGainsReadOnly gains,
                        double weight)
   {
      joints.add(joint);
      desiredPositions.add(desiredPosition);
      desiredVelocities.add(desiredVelocity);
      feedForwardAccelerations.add(feedForwardAcceleration);
      weightsForSolver.add(weight);
      this.gains.add().set(gains);
   }

   /**
    * Updates the desireds for a joint already registered given its index.
    *
    * @param jointIndex the index of the joint for which the desireds are to be updated.
    * @param desiredPosition the new joint's desired position.
    * @param desiredVelocity the new joint's desired velocity.
    * @param feedForwardAcceleration the new feed-forward acceleration for the joint, used to improve
    *           tracking performance.
    */
   public void setOneDoFJoint(int jointIndex, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredPositions.set(jointIndex, desiredPosition);
      desiredVelocities.set(jointIndex, desiredVelocity);
      feedForwardAccelerations.set(jointIndex, feedForwardAcceleration);
   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not modified.
    */
   @Override
   public void set(JointspaceFeedbackControlCommand other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         desiredPositions.add(other.getDesiredPosition(i));
         desiredVelocities.add(other.getDesiredVelocity(i));
         feedForwardAccelerations.add(other.getFeedForwardAcceleration(i));
         weightsForSolver.add(other.getWeightForSolver(i));
         gains.add().set(other.getGains(i));
      }
   }

   /**
    * Gets the number of registered joints in this command.
    *
    * @return the number of joints in this command.
    */
   public int getNumberOfJoints()
   {
      return joints.size();
   }

   /**
    * Gets the PD gains to be used for controlling the joints contained in this command.
    *
    * @param jointIndex the index of the joint gains.
    * @return the PD gains to be used.
    */
   public PDGainsReadOnly getGains(int jointIndex)
   {
      return gains.get(jointIndex);
   }

   /**
    * Retrieves the weight to be used in the QP optimization for a specific joint.
    *
    * @param jointIndex the index of the joint weight.
    * @return the weight to be used for the given joint in this command.
    */
   public double getWeightForSolver(int jointIndex)
   {
      return weightsForSolver.get(jointIndex);
   }

   /**
    * Retrieves a joint to be controlled from this command given its index.
    *
    * @param jointIndex the index of the joint to return.
    * @return a joint to be controlled.
    */
   public OneDoFJointBasics getJoint(int jointIndex)
   {
      return joints.get(jointIndex);
   }

   /**
    * Retrieves the joint's desired position given the joint index.
    *
    * @param jointIndex the index of the joint.
    * @return the desired position for the joint.
    */
   public double getDesiredPosition(int jointIndex)
   {
      return desiredPositions.get(jointIndex);
   }

   /**
    * Retrieves the joint's desired velocity given the joint index.
    *
    * @param jointIndex the index of the joint.
    * @return the desired velocity for the joint.
    */
   public double getDesiredVelocity(int jointIndex)
   {
      return desiredVelocities.get(jointIndex);
   }

   /**
    * Retrieves the joint's feed-forward acceleration given the joint index.
    *
    * @param jointIndex the index of the joint.
    * @return the feed-forward acceleration for the joint.
    */
   public double getFeedForwardAcceleration(int jointIndex)
   {
      return feedForwardAccelerations.get(jointIndex);
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
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof JointspaceFeedbackControlCommand)
      {
         JointspaceFeedbackControlCommand other = (JointspaceFeedbackControlCommand) object;

         if (getNumberOfJoints() != other.getNumberOfJoints())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJoints(); jointIndex++)
         {
            if (joints.get(jointIndex) != other.joints.get(jointIndex))
               return false;
            if (desiredPositions.get(jointIndex) != other.desiredPositions.get(jointIndex))
               return false;
            if (desiredVelocities.get(jointIndex) != other.desiredVelocities.get(jointIndex))
               return false;
            if (feedForwardAccelerations.get(jointIndex) != other.feedForwardAccelerations.get(jointIndex))
               return false;
            if (weightsForSolver.get(jointIndex) != other.weightsForSolver.get(jointIndex))
               return false;
            if (!gains.get(jointIndex).equals(other.getGains(jointIndex)))
               return false;
         }

         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * Builds and returns a representative {@code String} of this command.
    * <p>
    * It essentially represents the command as the name list of the joints to be controlled.
    * </p>
    *
    * @return the representative {@code String} for this command.
    */
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
