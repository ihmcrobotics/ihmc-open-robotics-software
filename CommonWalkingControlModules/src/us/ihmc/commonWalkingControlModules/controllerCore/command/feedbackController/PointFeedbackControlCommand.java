package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.PositionPIDGains;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * {@link PointFeedbackControlCommand} is a command meant to be submit to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link PointFeedbackControlCommand} is to notify the feedback controller
 * dedicated to control the end-effector provided in {@link #set(RigidBody, RigidBody)} that it is
 * requested to run during the next control tick.
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
public class PointFeedbackControlCommand implements FeedbackControlCommand<PointFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3D desiredPositionInWorld = new Point3D();
   private final Vector3D desiredLinearVelocityInWorld = new Vector3D();
   private final Vector3D feedForwardLinearAccelerationInWorld = new Vector3D();

   private final PositionPIDGains gains = new PositionPIDGains();

   private final PointAccelerationCommand pointAccelerationCommand = new PointAccelerationCommand();

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public PointFeedbackControlCommand()
   {
      pointAccelerationCommand.setSelectionMatrixToIdentity();
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(PointFeedbackControlCommand other)
   {
      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredLinearVelocityInWorld.set(other.desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationInWorld.set(other.feedForwardLinearAccelerationInWorld);
      setGains(other.gains);

      pointAccelerationCommand.set(other.pointAccelerationCommand);
   }

   /**
    * Specifies the rigid-body to be controlled, i.e. {@code endEffector}.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints
    * that can be to control the end-effector.
    * </p>
    * 
    * @param base the rigid-body located right before the first joint to be used for controlling the
    *           end-effector.
    * @param endEffector the rigid-body to be controlled.
    */
   public void set(RigidBody base, RigidBody endEffector)
   {
      pointAccelerationCommand.set(base, endEffector);
   }

   /**
    * Intermediate base located between the {@code base} and {@code endEffector}.
    * <p>
    * This parameter is optional. If provided, it is used to improve singularity avoidance by
    * applying a privileged joint configuration to the kinematic chain going from
    * {@code primaryBase} to {@code endEffector}.
    * </p>
    * <p>
    * Here is an example of application: {@code endEffector == leftHand},
    * {@code base == rootJoint.getPredecessor()} such that to control the {@code leftHand}, the
    * controller core uses the arm joints, the spine joints, and also the non-actuated floating
    * joint. If {@code primaryBase == chest}, as soon as the left arm comes close to a singular
    * configuration such as a straight elbow, the privileged configuration framework will help
    * bending the elbow. This reduces the time needed to escape the singular configuration. It also
    * prevents unfortunate situation where the elbow would try to bend past the joint limit.
    * </p>
    * 
    * @param primaryBase
    */
   public void setPrimaryBase(RigidBody primaryBase)
   {
      pointAccelerationCommand.setPrimaryBase(primaryBase);
   }

   /**
    * Sets the gains to use during the next control tick.
    * 
    * @param gains the new set of gains to use. Not modified.
    */
   public void setGains(PositionPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code bodyFixedPoint} provided.
    * </p>
    * <p>
    * The desired linear velocity and feed-forward linear acceleration are set to zero.
    * </p>
    * 
    * @param desiredPosition describes the position that the {@code bodyFixedPoint} should reach. It
    *           does NOT describe the desired position of {@code endEffector.getBodyFixedFrame()}.
    *           Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FramePoint desiredPosition)
   {
      desiredPosition.checkReferenceFrameMatch(worldFrame);

      desiredPosition.get(desiredPositionInWorld);
      desiredLinearVelocityInWorld.setToZero();
      feedForwardLinearAccelerationInWorld.setToZero();
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * WARNING: The information provided has to be relevant to the {@code bodyFixedPoint} provided.
    * </p>
    * 
    * @param desiredPosition describes the position that the {@code bodyFixedPoint} should reach. It
    *           does NOT describe the desired position of {@code endEffector.getBodyFixedFrame()}.
    *           Not modified.
    * @param desiredLinearVelocity describes the desired linear velocity of the
    *           {@code bodyFixedPoint} with respect to the {@code base}. It does NOT describe the
    *           desired linear velocity of {@code endEffector.getBodyFixedFrame()}'s origin. Not
    *           modified.
    * @param feedForwardLinearAcceleration describes the desired linear acceleration of the
    *           {@code bodyFixedPoint} with respect to the {@code base}. It does NOT describe the
    *           desired linear acceleration of {@code endEffector.getBodyFixedFrame()}'s origin. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if any of the three arguments is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FramePoint desiredPosition, FrameVector desiredLinearVelocity, FrameVector feedForwardLinearAcceleration)
   {
      desiredPosition.checkReferenceFrameMatch(worldFrame);
      desiredLinearVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardLinearAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredPosition.get(desiredPositionInWorld);
      desiredLinearVelocity.get(desiredLinearVelocityInWorld);
      feedForwardLinearAcceleration.get(feedForwardLinearAccelerationInWorld);
   }

   /**
    * Zeroes the offset of the {@code bodyFixedPoint} such that after calling this method <br>
    * {@code bodyFixedPoint == new FramePoint(endEffector.getBodyFixedFrame())}.
    */
   public void resetBodyFixedPoint()
   {
      pointAccelerationCommand.resetBodyFixedPoint();
   }

   /**
    * Sets the position of the {@code bodyFixedPoint} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code bodyFixedPoint} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame}
    * to the given desired position.
    * </p>
    * 
    * @param bodyFixedPointInEndEffectorFrame the position of the {@code bodyFixedPoint}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if any the argument is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setBodyFixedPointToControl(FramePoint bodyFixedPointInEndEffectorFrame)
   {
      pointAccelerationCommand.setBodyFixedPointToControl(bodyFixedPointInEndEffectorFrame);
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 3-by-3 identity matrix.
    * <p>
    * This specifies that the 3 translational degrees of freedom of the end-effector are to be
    * controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      pointAccelerationCommand.setSelectionMatrixToIdentity();
   }

   /**
    * Sets the selection matrix to be used for the next control tick.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector
    * that are to be controlled. A 3-by-3 identity matrix will request the control of all the 3
    * translational degrees of freedom.
    * </p>
    * <p>
    * Removing a row to the selection matrix using for instance
    * {@link MatrixTools#removeRow(DenseMatrix64F, int)} is the quickest way to ignore a specific
    * DoF of the end-effector.
    * </p>
    * <p>
    * 
    * @param selectionMatrix the new selection matrix to be used. Not modified.
    * @throws RuntimeException if the selection matrix has a number of rows greater than 3 or has a
    *            number of columns different to 3.
    */
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      pointAccelerationCommand.setSelectionMatrix(selectionMatrix);
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
      pointAccelerationCommand.setWeight(weight);
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
      pointAccelerationCommand.setWeights(weight);
   }

   public void getIncludingFrame(FramePoint desiredPositionToPack, FrameVector desiredLinearVelocityToPack, FrameVector feedForwardLinearAccelerationToPack)
   {
      desiredPositionToPack.setIncludingFrame(worldFrame, desiredPositionInWorld);
      desiredLinearVelocityToPack.setIncludingFrame(worldFrame, desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationToPack.setIncludingFrame(worldFrame, feedForwardLinearAccelerationInWorld);
   }

   public void getBodyFixedPointIncludingFrame(FramePoint bodyFixedPointToControlToPack)
   {
      pointAccelerationCommand.getBodyFixedPointIncludingFrame(bodyFixedPointToControlToPack);
   }

   public RigidBody getBase()
   {
      return pointAccelerationCommand.getBase();
   }

   public RigidBody getEndEffector()
   {
      return pointAccelerationCommand.getEndEffector();
   }

   public PointAccelerationCommand getPointAccelerationCommand()
   {
      return pointAccelerationCommand;
   }

   public PositionPIDGainsInterface getGains()
   {
      return gains;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.POINT;
   }
}
