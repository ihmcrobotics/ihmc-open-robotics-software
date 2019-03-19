package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

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
   // TODO: This is not used by the controller core. The control point orientation is only used with the spatial control command.
   private final FrameQuaternion bodyFixedOrientationInEndEffectorFrame = new FrameQuaternion();

   /** The end-effector's desired orientation expressed in root frame. */
   private final FrameQuaternion desiredOrientationInRootFrame = new FrameQuaternion();
   /** The end-effector's desired angular velocity expressed in root frame. */
   private final FrameVector3D desiredAngularVelocityInRootFrame = new FrameVector3D();
   /** The feed-forward to be used for the end-effector. Useful to improve tracking performance. */
   private final FrameVector3D feedForwardAngularActionInRootFrame = new FrameVector3D();

   /** The 3D gains used in the PD controller for the next control tick. */
   private final PID3DGains gains = new DefaultPID3DGains();
   /**
    * This is the reference frame in which the angular part of the gains are to be applied. If
    * {@code null}, it is applied in the control frame.
    */
   private ReferenceFrame angularGainsFrame = null;

   /**
    * Acceleration command used to save different control properties such as: the end-effector, the
    * base, and the weight to be used in the QP optimization.
    * <p>
    * Should not be accessed from the user side.
    * </p>
    */
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   /**
    * The control base frame is the reference frame with respect to which the end-effector is to be
    * controlled. More specifically, the end-effector desired velocity is assumed to be with respect to
    * the control base frame.
    */
   private ReferenceFrame controlBaseFrame = null;

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
      desiredOrientationInRootFrame.setIncludingFrame(other.desiredOrientationInRootFrame);
      desiredAngularVelocityInRootFrame.setIncludingFrame(other.desiredAngularVelocityInRootFrame);
      feedForwardAngularActionInRootFrame.setIncludingFrame(other.feedForwardAngularActionInRootFrame);
      gains.set(other.gains);

      spatialAccelerationCommand.set(other.spatialAccelerationCommand);

      resetBodyFixedOrientation();
      setBodyFixedOrientationToControl(other.getBodyFixedOrientationToControl());

      controlBaseFrame = other.controlBaseFrame;
   }

   /**
    * Specifies the rigid-body to be controlled, i.e. {@code endEffector}.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be to control the end-effector.
    * </p>
    *
    * @param base the rigid-body located right before the first joint to be used for controlling the
    *           end-effector.
    * @param endEffector the rigid-body to be controlled.
    */
   public void set(RigidBodyBasics base, RigidBodyBasics endEffector)
   {
      spatialAccelerationCommand.set(base, endEffector);
      resetBodyFixedOrientation();
   }

   /**
    * Intermediate base located between the {@code base} and {@code endEffector}.
    * <p>
    * This parameter is optional. If provided, it is used to improve singularity avoidance by applying
    * a privileged joint configuration to the kinematic chain going from {@code primaryBase} to
    * {@code endEffector}.
    * </p>
    * <p>
    * Here is an example of application: {@code endEffector == leftHand},
    * {@code base == rootJoint.getPredecessor()} such that to control the {@code leftHand}, the
    * controller core uses the arm joints, the spine joints, and also the non-actuated floating joint.
    * If {@code primaryBase == chest}, as soon as the left arm comes close to a singular configuration
    * such as a straight elbow, the privileged configuration framework will help bending the elbow.
    * This reduces the time needed to escape the singular configuration. It also prevents unfortunate
    * situation where the elbow would try to bend past the joint limit.
    * </p>
    *
    * @param primaryBase
    */
   public void setPrimaryBase(RigidBodyBasics primaryBase)
   {
      spatialAccelerationCommand.setPrimaryBase(primaryBase);
   }

   /**
    * The control base frame is the reference frame with respect to which the end-effector is to be
    * controlled. More specifically, the end-effector desired velocity is assumed to be with respect to
    * the control base frame.
    *
    * @param controlBaseFrame the new control base frame.
    */
   public void setControlBaseFrame(ReferenceFrame controlBaseFrame)
   {
      if (controlBaseFrame == getBase().getBodyFixedFrame())
         this.controlBaseFrame = null;
      else if (controlBaseFrame.isAStationaryFrame() || controlBaseFrame instanceof MovingReferenceFrame)
         this.controlBaseFrame = controlBaseFrame;
      else
         throw new IllegalArgumentException("The control base frame has to either be a stationary frame or a MovingReferenceFrame.");
   }

   /**
    * Sets the gains to use during the next control tick.
    *
    * @param gains the new set of gains to use. Not modified.
    */
   public void setGains(PID3DGainsReadOnly gains)
   {
      this.gains.set(gains);
   }

   /**
    * Resets the control base frame to its default value.
    *
    * @see #setControlBaseFrame(ReferenceFrame)
    */
   public void resetControlBaseFrame()
   {
      controlBaseFrame = null;
   }

   /**
    * Sets the reference frame in which the gains should be applied.
    * <p>
    * If the reference frame is {@code null}, the gains will be applied in the control frame.
    * </p>
    *
    * @param angularGainsFrame the reference frame to use for the orientation gains.
    */
   public void setGainsFrame(ReferenceFrame angularGainsFrame)
   {
      this.angularGainsFrame = angularGainsFrame;
   }

   /**
    * Sets the desired data expressed in root frame to be used during the next control tick.
    * <p>
    * The desired angular velocity and feed-forward angular acceleration are set to zero.
    * </p>
    *
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Not modified.
    */
   public void set(FrameQuaternionReadOnly desiredOrientation)
   {
      ReferenceFrame rootFrame = desiredOrientation.getReferenceFrame().getRootFrame();
      desiredOrientationInRootFrame.setIncludingFrame(desiredOrientation);
      desiredOrientationInRootFrame.changeFrame(rootFrame);
      desiredAngularVelocityInRootFrame.setToZero(rootFrame);
      feedForwardAngularActionInRootFrame.setToZero(rootFrame);
   }

   /**
    * Sets the desired data expressed in root frame to be used during the next control tick.
    *
    * @param desiredOrientation describes the orientation that the
    *           {@code endEffector.getBodyFixedFrame()} should reach. Not modified.
    * @param desiredAngularVelocity describes the desired linear velocity of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not modified.
    */
   public void set(FrameQuaternionReadOnly desiredOrientation, FrameVector3DReadOnly desiredAngularVelocity)
   {
      ReferenceFrame rootFrame = desiredOrientation.getReferenceFrame().getRootFrame();
      desiredOrientationInRootFrame.setIncludingFrame(desiredOrientation);
      desiredOrientationInRootFrame.changeFrame(rootFrame);
      desiredAngularVelocityInRootFrame.setIncludingFrame(desiredAngularVelocity);
      desiredAngularVelocityInRootFrame.changeFrame(rootFrame);
      feedForwardAngularActionInRootFrame.setToZero(rootFrame);
   }

   /**
    * Sets the desired feed forward data expressed in root frame to be used during the next control
    * tick.
    *
    * @param feedForwardAngularAction describes the desired linear action of
    *           {@code endEffector.getBodyFixedFrame()} with respect to the {@code base}. Not modified.
    */
   public void setFeedForwardAction(FrameVector3DReadOnly feedForwardAngularAction)
   {
      ReferenceFrame rootFrame = feedForwardAngularAction.getReferenceFrame().getRootFrame();
      feedForwardAngularActionInRootFrame.setIncludingFrame(feedForwardAngularAction);
      feedForwardAngularActionInRootFrame.changeFrame(rootFrame);
   }

   /**
    * Zeroes the offset of the {@code bodyFixedOrientation} such that after calling this method <br>
    * {@code bodyFixedOrientation == new FrameQuaternion(endEffector.getBodyFixedFrame())}.
    */
   public void resetBodyFixedOrientation()
   {
      bodyFixedOrientationInEndEffectorFrame.setToZero(getEndEffector().getBodyFixedFrame());
   }

   /**
    * Sets the position of the {@code bodyFixedOrientation} with respect to the
    * {@code endEffector.getBodyFixedFrame()}.
    * <p>
    * The {@code bodyFixedOrientation} describes on what the feedback control is applied, such that the
    * feedback controller for this end-effector will do its best to bring the {@code controlFrame} to
    * the given desired orientation.
    * </p>
    *
    * @param bodyFixedOrientationInEndEffectorFrame the position of the {@code bodyFixedOrientation}.
    *           Not modified.
    * @throws ReferenceFrameMismatchException if any the argument is not expressed in
    *            {@code endEffector.getBodyFixedFrame()}.
    */
   public void setBodyFixedOrientationToControl(FrameQuaternionReadOnly bodyFixedOrientationInEndEffectorFrame)
   {
      bodyFixedOrientationInEndEffectorFrame.checkReferenceFrameMatch(getEndEffector().getBodyFixedFrame());
      this.bodyFixedOrientationInEndEffectorFrame.set(bodyFixedOrientationInEndEffectorFrame);
   }

   /**
    * This specifies that the 3 rotational degrees of freedom of the end-effector are to be controlled.
    */
   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixForAngularControl();
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
      spatialAccelerationCommand.setSelectionMatrixForAngularControl(selectionMatrix);
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
    * Sets the angular weights to use in the optimization problem for each individual degree of
    * freedom.
    * <p>
    * WARNING: It is not the value of each individual command's weight that is relevant to how the
    * optimization will behave but the ratio between them. A command with a higher weight than other
    * commands value will be treated as more important than the other commands.
    * </p>
    *
    * @param angularWeightMatrix weight matrix holding the angular weights to use for each component of
    *           the desired acceleration. Not modified.
    */
   public void setWeightMatrix(WeightMatrix3D angularWeightMatrix)
   {
      spatialAccelerationCommand.setAngularPartOfWeightMatrix(angularWeightMatrix);
      spatialAccelerationCommand.setLinearWeightsToZero();
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
    */
   public void setWeightsForSolver(Vector3DReadOnly angular)
   {
      spatialAccelerationCommand.setAngularWeights(angular);
      spatialAccelerationCommand.setLinearWeightsToZero();
   }

   public void getIncludingFrame(FrameQuaternion desiredOrientationToPack)
   {
      desiredOrientationToPack.setIncludingFrame(desiredOrientationInRootFrame);
   }

   public void getIncludingFrame(FrameQuaternion desiredOrientationToPack, FrameVector3D desiredAngularVelocityToPack)
   {
      desiredOrientationToPack.setIncludingFrame(desiredOrientationInRootFrame);
      desiredAngularVelocityToPack.setIncludingFrame(desiredAngularVelocityInRootFrame);
   }

   public void getFeedForwardActionIncludingFrame(FrameVector3D feedForwardAngularActionToPack)
   {
      feedForwardAngularActionToPack.setIncludingFrame(feedForwardAngularActionInRootFrame);
   }

   public void getBodyFixedOrientationIncludingFrame(FrameQuaternion bodyFixedOrientationToControlToPack)
   {
      bodyFixedOrientationToControlToPack.setIncludingFrame(bodyFixedOrientationInEndEffectorFrame);
   }

   public FrameQuaternionBasics getBodyFixedOrientationToControl()
   {
      return bodyFixedOrientationInEndEffectorFrame;
   }

   public FrameQuaternionBasics getDesiredOrientation()
   {
      return desiredOrientationInRootFrame;
   }

   public FrameVector3DBasics getDesiredAngularVelocity()
   {
      return desiredAngularVelocityInRootFrame;
   }

   public FrameVector3DBasics getFeedForwardAngularAction()
   {
      return feedForwardAngularActionInRootFrame;
   }

   public RigidBodyBasics getBase()
   {
      return spatialAccelerationCommand.getBase();
   }

   public RigidBodyBasics getEndEffector()
   {
      return spatialAccelerationCommand.getEndEffector();
   }

   public ReferenceFrame getControlBaseFrame()
   {
      if (controlBaseFrame != null)
         return controlBaseFrame;
      else
         return spatialAccelerationCommand.getBase().getBodyFixedFrame();
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

   public PID3DGains getGains()
   {
      return gains;
   }

   public ReferenceFrame getAngularGainsFrame()
   {
      return angularGainsFrame;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof OrientationFeedbackControlCommand)
      {
         OrientationFeedbackControlCommand other = (OrientationFeedbackControlCommand) object;

         if (!bodyFixedOrientationInEndEffectorFrame.equals(other.bodyFixedOrientationInEndEffectorFrame))
            return false;
         if (!desiredOrientationInRootFrame.equals(other.desiredOrientationInRootFrame))
            return false;
         if (!desiredAngularVelocityInRootFrame.equals(other.desiredAngularVelocityInRootFrame))
            return false;
         if (!feedForwardAngularActionInRootFrame.equals(other.feedForwardAngularActionInRootFrame))
            return false;
         if (!gains.equals(other.gains))
            return false;
         if (angularGainsFrame != other.angularGainsFrame)
            return false;
         if (!spatialAccelerationCommand.equals(other.spatialAccelerationCommand))
            return false;
         if (controlBaseFrame != other.controlBaseFrame)
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
      ret += "base = " + spatialAccelerationCommand.getBase().getName() + ", ";
      ret += "endEffector = " + spatialAccelerationCommand.getEndEffector().getName() + ", ";
      ret += "orientation = " + desiredOrientationInRootFrame.toStringAsYawPitchRoll();
      return ret;
   }
}
