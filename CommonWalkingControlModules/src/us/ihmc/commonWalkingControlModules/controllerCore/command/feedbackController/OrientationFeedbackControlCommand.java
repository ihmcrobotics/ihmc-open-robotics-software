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
    * Sets the end-effector to be controlled.
    * <p>
    * The path from {@code base} to {@code endEffector} indicates the joints that can be used to
    * control the end-effector.
    * </p>
    * 
    * @param base the rigid-body from which joints are used to control the end-effector.
    * @param endEffector the rigid-body to be controlled.
    */
   public void set(RigidBody base, RigidBody endEffector)
   {
      spatialAccelerationCommand.set(base, endEffector);
   }

   public void setPrimaryBase(RigidBody primaryBase)
   {
      spatialAccelerationCommand.setPrimaryBase(primaryBase);
   }

   public void setGains(OrientationPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   public void set(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.checkReferenceFrameMatch(worldFrame);
      desiredAngularVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardAngularAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocity.get(desiredAngularVelocityInWorld);
      feedForwardAngularAcceleration.get(feedForwardAngularAccelerationInWorld);
   }

   public void changeFrameAndSet(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      feedForwardAngularAcceleration.changeFrame(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocity.get(desiredAngularVelocityInWorld);
      feedForwardAngularAcceleration.get(feedForwardAngularAccelerationInWorld);
   }

   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > 3)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());

      spatialAccelerationCommand.setSelectionMatrix(selectionMatrix);
   }

   public void setWeightForSolver(double weight)
   {
      spatialAccelerationCommand.setWeight(weight);
   }

   public void setWeightsForSolver(Vector3D weight)
   {
      spatialAccelerationCommand.setAngularWeights(weight);
      spatialAccelerationCommand.setLinearWeightsToZero();
   }

   public void setAlphaTaskPriorityForSolver(double alpha)
   {
      spatialAccelerationCommand.setAlphaTaskPriority(alpha);
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
