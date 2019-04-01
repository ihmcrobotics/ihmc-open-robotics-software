package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;

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
public class JointspaceFeedbackControlCommand extends FeedbackControlCommandList
{
   /** Initial capacity of the internal memory. */
   private static final int initialCapacity = 15;
   /** Gains to used by the PD controllers for the next control tick. */
   private final RecyclingArrayList<OneDoFJointFeedbackControlCommand> jointCommands = new RecyclingArrayList<>(initialCapacity,
                                                                                                                OneDoFJointFeedbackControlCommand.class);

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
   @Override
   public void clear()
   {
      super.clear();
      jointCommands.clear();
   }

   /**
    * Adds a {@code OneDoFJointFeedbackControlCommand} to this command.
    *
    * @throws IllegalArgumentException if the given {@code command} is not an instance of
    *            {@code OneDoFJointFeedbackControlCommand}.
    */
   @Override
   public void addCommand(FeedbackControlCommand<?> command)
   {
      if (command instanceof OneDoFJointFeedbackControlCommand)
         addCommand((OneDoFJointFeedbackControlCommand) command);
      else
         throw new IllegalArgumentException("Cannot add a " + command.getClass().getSimpleName() + " to a " + getClass().getSimpleName());
   }

   /**
    * Adds a joint command to this command.
    *
    * @param command the joint command to be registered. Not modified.
    */
   public void addCommand(OneDoFJointFeedbackControlCommand command)
   {
      addEmptyCommand().set(command);
   }

   @Override
   public FeedbackControlCommand<?> pollCommand()
   {
      throw new UnsupportedOperationException();
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
         jointCommands.get(jointIndex).setGains(gains);
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
         jointCommands.get(jointIndex).setWeightForSolver(weight);
   }

   /**
    * Adds an empty joint command to this list.
    * <p>
    * The new command has to be configured.
    * </p>
    *
    * @return the new empty command.
    */
   public OneDoFJointFeedbackControlCommand addEmptyCommand()
   {
      OneDoFJointFeedbackControlCommand command = jointCommands.add();
      command.clear();
      super.addCommand(command);
      return command;
   }

   /**
    * Adds a joint to be controlled in the next control tick.
    *
    * @param joint the joint to be controlled.
    */
   public OneDoFJointFeedbackControlCommand addJointCommand(OneDoFJointBasics joint)
   {
      OneDoFJointFeedbackControlCommand jointCommand = addEmptyCommand();
      jointCommand.setJoint(joint);
      return jointCommand;
   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not modified.
    * @throws IllegalArgumentException if {@code other} is not an instance of
    *            {@code JointspaceFeedbackControlCommand}.
    */
   @Override
   public void set(FeedbackControlCommandList other)
   {
      if (other instanceof JointspaceFeedbackControlCommand)
         set((JointspaceFeedbackControlCommand) other);
      else
         throw new IllegalArgumentException("Cannot set a " + getClass().getSimpleName() + " from a " + other.getClass().getSimpleName());

   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not modified.
    */
   public void set(JointspaceFeedbackControlCommand other)
   {
      clear();
      for (int jointIndex = 0; jointIndex < other.getNumberOfCommands(); jointIndex++)
      {
         addCommand(other.jointCommands.get(jointIndex));
      }
   }

   /**
    * Gets the number of registered joints in this command.
    *
    * @return the number of joints in this command.
    */
   public int getNumberOfJoints()
   {
      return getNumberOfCommands();
   }

   /**
    * Retrieves a joint to be controlled from this command given its index.
    *
    * @param jointIndex the index of the joint to return.
    * @return a joint to be controlled.
    */
   public OneDoFJointBasics getJoint(int jointIndex)
   {
      return jointCommands.get(jointIndex).getJoint();
   }

   /**
    * Updates the desireds for a joint already registered given its index.
    *
    * @param jointIndex the index of the joint for which the desireds are to be updated.
    */
   public OneDoFJointFeedbackControlCommand getJointCommand(int jointIndex)
   {
      return jointCommands.get(jointIndex);
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
            if (!jointCommands.get(jointIndex).equals(other.jointCommands.get(jointIndex)))
               return false;
         }

         return super.equals(object);
      }
      else
      {
         return false;
      }
   }
}
