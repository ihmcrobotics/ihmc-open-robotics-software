package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDGainsInterface;
import us.ihmc.robotics.controllers.SimplePDGainsHolder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * A {@code JointspaceFeedbackControlCommand} can be used to request the
 * {@link WholeBodyFeedbackController} to run PD controllers on a list of {@link OneDoFJoint}s to
 * reach given desired positions and desired velocities.
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
   private final List<OneDoFJoint> joints = new ArrayList<>(initialCapacity);
   /**
    * Internal memory to save the names of the joints to be controlled. This is used when passing
    * the command between two modules using different instances of the same physical robot.
    */
   private final List<String> jointNames = new ArrayList<>(initialCapacity);

   /** The desired positions for each controlled joint. */
   private final RecyclingArrayList<MutableDouble> desiredPositions = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   /** The desired velocities for each controlled joint. */
   private final RecyclingArrayList<MutableDouble> desiredVelocities = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);
   /**
    * The feed-forward to be used for each controlled joint. Useful to improve tracking performance.
    */
   private final RecyclingArrayList<MutableDouble> feedForwardAccelerations = new RecyclingArrayList<>(initialCapacity, MutableDouble.class);

   /** Gains to used by the PD controllers for the next control tick. */
   private final SimplePDGainsHolder gains = new SimplePDGainsHolder();
   /** Weight used in the QP optimization describing how 'important' achieving this command is. */
   private double weightForSolver = Double.POSITIVE_INFINITY;

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
      jointNames.clear();
      desiredPositions.clear();
      desiredVelocities.clear();
      feedForwardAccelerations.clear();
   }

   /**
    * Updates the gains to be used for the next control tick.
    *
    * @param gains the new set of gains to be used. The data is copied into a local object. Not
    *           modified.
    */
   public void setGains(PDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   /**
    * Updates the weight to be used for the next control tick.
    * <p>
    * This relates to how 'important' is this command compared to other commands submitted.
    * </p>
    *
    * @param weight the new weight for the QP optimization.
    */
   public void setWeightForSolver(double weight)
   {
      weightForSolver = weight;
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
   public void addJoint(OneDoFJoint joint, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      joints.add(joint);
      jointNames.add(joint.getName());
      desiredPositions.add().setValue(desiredPosition);
      desiredVelocities.add().setValue(desiredVelocity);
      feedForwardAccelerations.add().setValue(feedForwardAcceleration);
   }

   /**
    * Updates the desireds for a joint already registered given its index.
    *
    * @param jointIndex the index of the joint for which the desireds are to be updated.
    * @param desiredPosition the new joint's desired position.
    * @param desiredVelocity the new joint's desired velocity.
    * @param feedForwardAcceleration the new feed-forward acceleration for the joint, used to
    *           improve tracking performance.
    */
   public void setOneDoFJoint(int jointIndex, double desiredPosition, double desiredVelocity, double feedForwardAcceleration)
   {
      MathTools.checkEquals(joints.get(jointIndex).getDegreesOfFreedom(), 1);
      desiredPositions.get(jointIndex).setValue(desiredPosition);
      desiredVelocities.get(jointIndex).setValue(desiredVelocity);
      feedForwardAccelerations.get(jointIndex).setValue(feedForwardAcceleration);
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
      gains.set(other.gains);
      weightForSolver = other.weightForSolver;

      for (int i = 0; i < other.getNumberOfJoints(); i++)
      {
         joints.add(other.joints.get(i));
         jointNames.add(other.jointNames.get(i));
         desiredPositions.add().setValue(other.getDesiredPosition(i));
         desiredVelocities.add().setValue(other.getDesiredVelocity(i));
         feedForwardAccelerations.add().setValue(other.getFeedForwardAcceleration(i));
      }
   }

   /**
    * Updates the internal references to the controlled joints by using a name map.
    * <p>
    * This is particularly useful when passing the command between two modules using different
    * instances of the same physical robot.
    * </p>
    *
    * @param nameToJointMap the map from joint name to joint reference of the entire robot.
    */
   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < getNumberOfJoints(); i++)
      {
         joints.set(i, nameToJointMap.get(jointNames.get(i)));
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
    * @return the PD gains to be used.
    */
   public PDGainsInterface getGains()
   {
      return gains;
   }

   /**
    * Retrieves the weight to be used in the QP optimization.
    *
    * @return the weight to be used for all the joints in this command.
    */
   public double getWeightForSolver()
   {
      return weightForSolver;
   }

   /**
    * Retrieves a joint to be controlled from this command given its index.
    *
    * @param jointIndex the index of the joint to return.
    * @return a joint to be controlled.
    */
   public OneDoFJoint getJoint(int jointIndex)
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
      return desiredPositions.get(jointIndex).doubleValue();
   }

   /**
    * Retrieves the joint's desired velocity given the joint index.
    *
    * @param jointIndex the index of the joint.
    * @return the desired velocity for the joint.
    */
   public double getDesiredVelocity(int jointIndex)
   {
      return desiredVelocities.get(jointIndex).doubleValue();
   }

   /**
    * Retrieves the joint's feed-forward acceleration given the joint index.
    *
    * @param jointIndex the index of the joint.
    * @return the feed-forward acceleration for the joint.
    */
   public double getFeedForwardAcceleration(int jointIndex)
   {
      return feedForwardAccelerations.get(jointIndex).doubleValue();
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
