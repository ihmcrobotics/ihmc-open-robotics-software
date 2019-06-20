package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

/**
 * {@link CenterOfMassFeedbackControlCommand} is a command meant to be submit to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link CenterOfMassFeedbackControlCommand} is to notify the center of mass
 * feedback controller that it is requested to run during the next control tick.
 * </p>
 * <p>
 * From control tick to control tick each feedback controller can be entirely configured or
 * reconfigured, and enabled (by submitting a command) or disabled (by NOT submitting a command).
 * </p>
 * <p>
 * All the data contained in this command is expressed in root frame to ensure that the feedback
 * controller can properly interpret it.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class CenterOfMassFeedbackControlCommand implements FeedbackControlCommand<CenterOfMassFeedbackControlCommand>
{
   /** Represents the expected control mode to execute this command. */
   private WholeBodyControllerCoreMode controlMode = null;
   /** The desired center of mass position to use in the feedback controller. */
   private final FramePoint3D referencePositionInRootFrame = new FramePoint3D();
   /**
    * The desired or (IK) feed-forward center of mass linear velocity to use in the feedback
    * controller.
    */
   private final FrameVector3D referenceLinearVelocityInRootFrame = new FrameVector3D();
   /** The (ID) feed-forward center of mass linear acceleration to use in the feedback controller. */
   private final FrameVector3D referenceLinearAccelerationInRootFrame = new FrameVector3D();
   /** The 3D gains used in the PD controller for the next control tick. */
   private final DefaultPID3DGains gains = new DefaultPID3DGains();

   /**
    * Momentum rate command used to save different control properties such as the weight to be used in
    * the QP optimization.
    * <p>
    * Should not be accessed from the user side.
    * </p>
    */
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public CenterOfMassFeedbackControlCommand()
   {
      momentumRateCommand.setSelectionMatrixForLinearControl();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(CenterOfMassFeedbackControlCommand other)
   {
      controlMode = other.controlMode;
      referencePositionInRootFrame.setIncludingFrame(other.referencePositionInRootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(other.referenceLinearVelocityInRootFrame);
      referenceLinearAccelerationInRootFrame.setIncludingFrame(other.referenceLinearAccelerationInRootFrame);
      setGains(other.gains);

      momentumRateCommand.set(other.momentumRateCommand);
   }

   /**
    * Sets the gains to use during the next control tick.
    *
    * @param gains the new set of gains to use. Not modified.
    */
   public void setGains(PID3DGains gains)
   {
      this.gains.set(gains);
   }

   /**
    * Sets the expected control mode that the controller core should be using to execute this command.
    * <p>
    * Note that the control mode is updated when calling either:
    * {@link #setInverseKinematics(FramePoint3DReadOnly, FrameVector3DReadOnly)},
    * {@link #setInverseDynamics(FramePoint3DReadOnly, FrameVector3DReadOnly, FrameVector3DReadOnly)},
    * or
    * {@link #setVirtualModelControl(FramePoint3DReadOnly, FrameVector3DReadOnly, FrameVector3DReadOnly)}.
    * </p>
    * <p>
    * This is a safety feature, the controller core will throw an exception in the case the control
    * mode mismatches the active mode of the controller core.
    * </p>
    * 
    * @param controlMode the expected control mode.
    */
   public void setControlMode(WholeBodyControllerCoreMode controlMode)
   {
      this.controlMode = controlMode;
   }

   /**
    * Configures this feedback command's inputs for inverse kinematics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for inverse kinematics.
    * </p>
    * <p>
    * The reference linear acceleration is set to zero.
    * </p>
    *
    * @param desiredPosition the position that the center of mass should reach. Not modified.
    * @param feedForwardLinearVelocity the feed-forward linear velocity with respect to root frame. Not
    *           modified.
    */
   public void setInverseKinematics(FramePoint3DReadOnly desiredPosition, FrameVector3DReadOnly feedForwardLinearVelocity)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
      ReferenceFrame rootFrame = desiredPosition.getReferenceFrame().getRootFrame();
      referencePositionInRootFrame.setIncludingFrame(desiredPosition);
      referencePositionInRootFrame.changeFrame(rootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(feedForwardLinearVelocity);
      referenceLinearVelocityInRootFrame.changeFrame(rootFrame);
      referenceLinearAccelerationInRootFrame.setToZero(rootFrame);
   }

   /**
    * Configures this feedback command's inputs for inverse dynamics.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control for inverse dynamics.
    * </p>
    *
    * @param desiredPosition the position that the center of mass should reach. Not modified.
    * @param desiredLinearVelocity the desired center of mass linear velocity with respect to root
    *           frame. Not modified.
    * @param feedForwardLinearAcceleration feed-forward linear acceleration with respect to root frame.
    *           Not modified.
    */
   public void setInverseDynamics(FramePoint3DReadOnly desiredPosition, FrameVector3DReadOnly desiredLinearVelocity,
                                  FrameVector3DReadOnly feedForwardLinearAcceleration)
   {
      setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      ReferenceFrame rootFrame = desiredPosition.getReferenceFrame().getRootFrame();
      referencePositionInRootFrame.setIncludingFrame(desiredPosition);
      referencePositionInRootFrame.changeFrame(rootFrame);
      referenceLinearVelocityInRootFrame.setIncludingFrame(desiredLinearVelocity);
      referenceLinearVelocityInRootFrame.changeFrame(rootFrame);
      referenceLinearAccelerationInRootFrame.setIncludingFrame(feedForwardLinearAcceleration);
      referenceLinearAccelerationInRootFrame.changeFrame(rootFrame);
   }

   /**
    * Configures this feedback command's inputs for virtual model control.
    * <p>
    * Sets the desired data expressed in root frame to be used during the next control tick and sets
    * the control mode for virtual model control.
    * </p>
    *
    * @param desiredPosition the position that the center of mass should reach. Not modified.
    * @param desiredLinearVelocity the desired center of mass linear velocity with respect to root
    *           frame. Not modified.
    * @param feedForwardLinearAcceleration feed-forward linear acceleration with respect to root frame.
    *           Not modified.
    */
   public void setVirtualModelControl(FramePoint3DReadOnly desiredPosition, FrameVector3DReadOnly desiredLinearVelocity,
                                      FrameVector3DReadOnly feedForwardLinearAcceleration)
   {
      setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      setControlMode(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
   }

   /**
    * This specifies that the 3 translational degrees of freedom of the center of mass are to be
    * controlled.
    */
   public void setSelectionMatrixToIdentity()
   {
      momentumRateCommand.setSelectionMatrixForLinearControl();
   }

   /**
    * Convenience method that sets up the selection matrix such that only the x and y components of the
    * linear part of this command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearXYControl()
   {
      momentumRateCommand.setSelectionMatrixForLinearXYControl();
   }

   /**
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector that
    * are to be controlled. It is initialized such that the controller will by default control all the
    * end-effector DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
    * </p>
    *
    * @param selectionMatrix the selection matrix to copy data from. Not modified.
    */
   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      momentumRateCommand.setSelectionMatrixForLinearControl(selectionMatrix);
   }

   /**
    * Sets the weight to use in the optimization problem.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param weight the weight value to use for this command.
    */
   public void setWeightForSolver(double weight)
   {
      momentumRateCommand.setWeight(weight);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param weightVector dense matrix holding the weights to use for each component of the desired
    *           center of mass position. It is expected to be a 3-by-1 vector ordered as:
    *           {@code linearX}, {@code linearY}, {@code linearZ}. Not modified.
    */
   public void setWeightsForSolver(DenseMatrix64F weightVector)
   {
      if (weightVector.getNumRows() != 3)
         throw new RuntimeException("Unexpected number of rows for the given weight vector. Expected 3 but was: " + weightVector.getNumRows());

      momentumRateCommand.setWeights(0.0, 0.0, 0.0, weightVector.get(0, 0), weightVector.get(1, 0), weightVector.get(2, 0));
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param weight the weight to use for each direction. Not modified.
    */
   public void setWeightsForSolver(Vector3D weight)
   {
      momentumRateCommand.setLinearWeights(weight);
      momentumRateCommand.setAngularWeightsToZero();
   }

   /**
    * Gets the expected control mode to execute this command with.
    * 
    * @return the expected active controller core control mode.
    */
   public WholeBodyControllerCoreMode getControlMode()
   {
      return controlMode;
   }

   /**
    * Gets the reference position to use in the feedback controller.
    * <p>
    * The reference position typically represents the desired position.
    * </p>
    * 
    * @return the reference position.
    */
   public FramePoint3DBasics getReferencePosition()
   {
      return referencePositionInRootFrame;
   }

   /**
    * Gets the reference linear velocity to use in the feedback controller.
    * <p>
    * Depending on the active control mode, it can be used as a desired or a feed-forward term.
    * </p>
    * 
    * @return the reference linear velocity.
    */
   public FrameVector3DBasics getReferenceLinearVelocity()
   {
      return referenceLinearVelocityInRootFrame;
   }

   /**
    * Gets the reference linear acceleration to use in the feedback controller.
    * <p>
    * Note that when the controller core is running in inverse kinematics mode, the reference linear
    * acceleration is not used.
    * </p>
    * 
    * @return the reference linear acceleration.
    */
   public FrameVector3DBasics getReferenceLinearAcceleration()
   {
      return referenceLinearAccelerationInRootFrame;
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   public PID3DGains getGains()
   {
      return gains;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.MOMENTUM;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof CenterOfMassFeedbackControlCommand)
      {
         CenterOfMassFeedbackControlCommand other = (CenterOfMassFeedbackControlCommand) object;

         if (controlMode != other.controlMode)
            return false;
         if (!referencePositionInRootFrame.equals(other.referencePositionInRootFrame))
            return false;
         if (!referenceLinearVelocityInRootFrame.equals(other.referenceLinearVelocityInRootFrame))
            return false;
         if (!referenceLinearAccelerationInRootFrame.equals(other.referenceLinearAccelerationInRootFrame))
            return false;
         if (!gains.equals(other.gains))
            return false;
         if (!momentumRateCommand.equals(other.momentumRateCommand))
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
      return getClass().getSimpleName() + ": control mode: " + controlMode + ", reference position: " + referencePositionInRootFrame + ", reference velocity: "
            + referenceLinearVelocityInRootFrame + ", reference acceleration: " + referenceLinearAccelerationInRootFrame;
   }
}
