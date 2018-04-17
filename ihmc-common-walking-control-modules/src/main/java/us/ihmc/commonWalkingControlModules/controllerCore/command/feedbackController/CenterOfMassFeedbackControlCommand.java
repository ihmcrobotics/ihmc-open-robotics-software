package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Point3D;
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
 * All the data contained in this command is expressed in world to ensure that the feedback
 * controller can properly interpret it.
 * </p>
 *
 * @author Sylvain Bertrand
 *
 */
public class CenterOfMassFeedbackControlCommand implements FeedbackControlCommand<CenterOfMassFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3D desiredPositionInWorld = new Point3D();
   private final Vector3D desiredLinearVelocityInWorld = new Vector3D();
   private final Vector3D feedForwardLinearAccelerationInWorld = new Vector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final PID3DGains gains = new DefaultPID3DGains();

   /**
    * Momentum rate command used to save different control properties such as the weight to be used
    * in the QP optimization.
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
      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredLinearVelocityInWorld.set(other.desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationInWorld.set(other.feedForwardLinearAccelerationInWorld);
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
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * The desired linear velocity and feed-forward linear acceleration are set to zero.
    * </p>
    *
    * @param desiredPosition describes the position that the center of mass should reach. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FramePoint3D desiredPosition)
   {
      desiredPosition.checkReferenceFrameMatch(worldFrame);

      desiredPositionInWorld.set(desiredPosition);
      desiredLinearVelocityInWorld.setToZero();
      feedForwardLinearAccelerationInWorld.setToZero();
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    *
    * @param desiredPosition describes the position that the center of mass should reach. Not
    *           modified.
    * @param desiredLinearVelocity describes the desired center of mass linear velocity with respect
    *           to world. Not modified.
    * @param feedForwardLinearAcceleration describes the desired center of mass linear acceleration
    *           with respect to the world. Not modified.
    * @throws ReferenceFrameMismatchException if any of the three arguments is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FramePoint3D desiredPosition, FrameVector3D desiredLinearVelocity, FrameVector3D feedForwardLinearAcceleration)
   {
      desiredPosition.checkReferenceFrameMatch(worldFrame);
      desiredLinearVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardLinearAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredPositionInWorld.set(desiredPosition);
      desiredLinearVelocityInWorld.set(desiredLinearVelocity);
      feedForwardLinearAccelerationInWorld.set(feedForwardLinearAcceleration);
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
    * Convenience method that sets up the selection matrix such that only the x and y components of
    * the linear part of this command will be considered in the optimization.
    */
   public void setSelectionMatrixForLinearXYControl()
   {
      momentumRateCommand.setSelectionMatrixForLinearXYControl();
   }

   /**
    * Sets this command's selection matrix to the given one.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector
    * that are to be controlled. It is initialized such that the controller will by default control
    * all the end-effector DoFs.
    * </p>
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the
    * selection frame is equal to the control frame.
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
    * @param weight dense matrix holding the weights to use for each component of the desired center
    *           of mass position. It is expected to be a 3-by-1 vector ordered as: {@code linearX},
    *           {@code linearY}, {@code linearZ}. Not modified.
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

   public void getIncludingFrame(FramePoint3D desiredPositionToPack, FrameVector3D desiredLinearVelocityToPack, FrameVector3D feedForwardLinearAccelerationToPack)
   {
      desiredPositionToPack.setIncludingFrame(worldFrame, desiredPositionInWorld);
      desiredLinearVelocityToPack.setIncludingFrame(worldFrame, desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationToPack.setIncludingFrame(worldFrame, feedForwardLinearAccelerationInWorld);
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

}
