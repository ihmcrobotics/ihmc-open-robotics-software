package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.controllers.OrientationPIDGains;
import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * A {@code OrientationFeedbackControlCommand} can be used to request the
 * {@link WholeBodyFeedbackController} to run a PD controller in that task space of an end-effector
 * given its desired orientation and angular velocity.
 * <p>
 * The PD controller used also handle a feed-forward angular acceleration for improved tracking
 * performance.
 * </p>
 * <p>
 * In addition to the desireds, the gains to be used in the PD controller have to be provided
 * alongside with the weight used in the QP optimization problem, see for instance
 * {@link WholeBodyInverseDynamicsSolver}.
 * </p>
 * Every control tick, a {@code OrientationFeedbackControlCommand} has to be sent to the controller
 * core allowing the higher-level controller to continuously update the desireds, gains, and weight
 * to use.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class OrientationFeedbackControlCommand implements FeedbackControlCommand<OrientationFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   /** The end-effector's desired orientation expressed in world frame. */
   private final Quaternion desiredOrientationInWorld = new Quaternion();
   /** The end-effector's desired angular velocity expressed in world frame. */
   private final Vector3D desiredAngularVelocityInWorld = new Vector3D();
   /** The feed-forward to be used for the end-effector. Useful to improve tracking performance. */
   private final Vector3D feedForwardAngularAccelerationInWorld = new Vector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final OrientationPIDGains gains = new OrientationPIDGains();

   /**
    * Acceleration command used to save different control properties such as: the end-effector, the
    * base, and the weight to be used in the QP optimization.
    * <p>
    * Should not be accessed from the user side.
    * </p>
    */
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   /**
    * Creates an empty command.
    */
   public OrientationFeedbackControlCommand()
   {
      spatialAccelerationCommand.setSelectionMatrixForAngularControl();
   }

   /**
    * Clears this command and then copies the data from {@code other} into this.
    *
    * @param other the other command to copy the data from. Not modified.
    */
   @Override
   public void set(OrientationFeedbackControlCommand other)
   {
      desiredOrientationInWorld.set(other.desiredOrientationInWorld);
      desiredAngularVelocityInWorld.set(other.desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationInWorld.set(other.feedForwardAngularAccelerationInWorld);
      gains.set(other.gains);

      spatialAccelerationCommand.set(other.spatialAccelerationCommand);
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
      spatialAccelerationCommand.set(base, endEffector);
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
      spatialAccelerationCommand.setPrimaryBase(primaryBase);
   }

   /**
    * Sets the gains to use during the next control tick.
    * 
    * @param gains the new set of gains to use. Not modified.
    */
   public void setGains(OrientationPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * <p>
    * The desired angular velocity and feed-forward angular acceleration are set to zero.
    * </p>
    * 
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FrameOrientation desiredOrientation)
   {
      desiredOrientation.checkReferenceFrameMatch(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocityInWorld.setToZero();
      feedForwardAngularAccelerationInWorld.setToZero();
   }

   /**
    * Sets the desired data expressed in world frame to be used during the next control tick.
    * 
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Not modified.
    * @param desiredAngularVelocity describes the desired linear velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not
    *           modified.
    * @param feedForwardAngularAcceleration describes the desired linear acceleration of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if any of the three arguments is not expressed in
    *            {@link ReferenceFrame#getWorldFrame()}.
    */
   public void set(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.checkReferenceFrameMatch(worldFrame);
      desiredAngularVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardAngularAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocity.get(desiredAngularVelocityInWorld);
      feedForwardAngularAcceleration.get(feedForwardAngularAccelerationInWorld);
   }

   /**
    * Change the reference frame of the given data such that it is expressed in
    * {@link ReferenceFrame#getWorldFrame()}. The data will be used for the next control tick.
    * 
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Modified.
    * @param desiredAngularVelocity describes the desired linear velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Modified.
    * @param feedForwardAngularAcceleration describes the desired linear acceleration of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Modified.
    */
   public void changeFrameAndSet(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      feedForwardAngularAcceleration.changeFrame(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocity.get(desiredAngularVelocityInWorld);
      feedForwardAngularAcceleration.get(feedForwardAngularAccelerationInWorld);
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the following 3-by-6 matrix:
    * 
    * <pre>
    *     / 1 0 0 0 0 0 \
    * S = | 0 1 0 0 0 0 |
    *     \ 0 0 1 0 0 0 /
    * </pre>
    * <p>
    * This specifies that the 3 rotational degrees of freedom of the end-effector are to be
    * controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
   }

   /**
    * Sets the selection matrix to be used for the next control tick.
    * <p>
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector
    * that are to be controlled. Using the follwing 3-by-6 matrix will request the control of all
    * the 3 rotational degrees of freedom:
    * 
    * <pre>
    *     / 1 0 0 0 0 0 \
    * S = | 0 1 0 0 0 0 |
    *     \ 0 0 1 0 0 0 /
    * </pre>
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
    *            number of columns different to 6.
    */
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > 3)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());

      spatialAccelerationCommand.setSelectionMatrix(selectionMatrix);
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
      spatialAccelerationCommand.setWeight(weight);
   }

   /**
    * Sets the weights to use in the optimization problem for each individual degree of freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    * 
    * @param angular the weights to use for the angular part of this command. Not modified.
    * @param linear the weight to use for the linear part of this command. Not modified.
    */
   public void setWeightsForSolver(Vector3D weight)
   {
      spatialAccelerationCommand.setAngularWeights(weight);
      spatialAccelerationCommand.setLinearWeightsToZero();
   }

   public void getIncludingFrame(FrameOrientation desiredOrientationToPack, FrameVector desiredAngularVelocityToPack,
                                 FrameVector feedForwardAngularAccelerationToPack)
   {
      desiredOrientationToPack.setIncludingFrame(worldFrame, desiredOrientationInWorld);
      desiredAngularVelocityToPack.setIncludingFrame(worldFrame, desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationToPack.setIncludingFrame(worldFrame, feedForwardAngularAccelerationInWorld);
   }

   public RigidBody getBase()
   {
      return spatialAccelerationCommand.getBase();
   }

   public RigidBody getEndEffector()
   {
      return spatialAccelerationCommand.getEndEffector();
   }

   public SpatialAccelerationCommand getSpatialAccelerationCommand()
   {
      return spatialAccelerationCommand;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.ORIENTATION;
   }

   public OrientationPIDGainsInterface getGains()
   {
      return gains;
   }
}
