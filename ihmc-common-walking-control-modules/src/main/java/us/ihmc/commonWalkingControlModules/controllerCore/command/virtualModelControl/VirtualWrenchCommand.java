package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace.SpatialFeedbackController;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

/**
 * {@link VirtualWrenchCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The objective of a {@link VirtualWrenchCommand} is to notify the virtual model control module
 * that the given end-effector is to track a desired wrench during the next control tick.
 * </p>
 * <p>
 * It is usually either the result of the {@link SpatialFeedbackController}.
 * </p>
 * 
 * @author Robert Griffin
 *
 */
public class VirtualWrenchCommand implements VirtualEffortCommand<VirtualWrenchCommand>
{
   /** Defines the reference frame of interest. It is attached to the end-effector. */
   private final FramePose3D controlFramePose = new FramePose3D();

   /**
    * It defines the desired linear force of the origin of the control frame, with respect to the base.
    * The vector is expressed in the control frame.
    */
   private final Vector3D desiredLinearForce = new Vector3D();
   /**
    * It defines the desired angular torque of the control frame, with respect to the base. The vector
    * is expressed in the control frame.
    */
   private final Vector3D desiredAngularTorque = new Vector3D();

   /**
    * The selection matrix is used to describe the DoFs (Degrees Of Freedom) of the end-effector that
    * are to be controlled. It is initialized such that the controller will by default control all the
    * end-effector DoFs.
    * <p>
    * If the selection frame is not set, it is assumed that the selection frame is equal to the control
    * frame.
    * </p>
    */
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   /**
    * The base is the rigid-body located right before the first joint to be used for controlling the
    * end-effector.
    */
   private RigidBodyBasics base;
   /** The end-effector is the rigid-body to be controlled. */
   private RigidBodyBasics endEffector;

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public VirtualWrenchCommand()
   {
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(VirtualWrenchCommand other)
   {
      controlFramePose.setIncludingFrame(other.controlFramePose);
      desiredLinearForce.set(other.desiredLinearForce);
      desiredAngularTorque.set(other.desiredAngularTorque);
      selectionMatrix.set(other.selectionMatrix);
      base = other.getBase();
      endEffector = other.getEndEffector();
   }

   /**
    * Copies all the fields of the given {@link SpatialAccelerationCommand} into this except for the
    * spatial acceleration or the weights.
    *
    * @param command the command to copy the properties from. Not modified.
    */
   public void setProperties(SpatialAccelerationCommand command)
   {
      command.getSelectionMatrix(selectionMatrix);
      base = command.getBase();
      endEffector = command.getEndEffector();

      command.getControlFramePoseIncludingFrame(controlFramePose);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
   }

   /**
    * Specifies the rigid-body to be controlled, i.e. {@code endEffector}.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be used to control the end-effector.
    * </p>
    * 
    * @param base the rigid-body located right before the first joint to be used for controlling the
    *           end-effector.
    * @param endEffector the rigid-body to be controlled.
    */
   public void set(RigidBodyBasics base, RigidBodyBasics endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;
   }

   /**
    * Sets the desired wrench to submit for the optimization to zero.
    * <p>
    * This is useful when the end-effector is in contact with the environment. Its wrench has to be set
    * to zero so it can exert the required wrench from the contact force optimization.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    */
   public void setWrenchToZero(ReferenceFrame controlFrame)
   {
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      desiredAngularTorque.setToZero();
      desiredLinearForce.setToZero();
   }

   /**
    * Sets the desired wrench to submit for the optimization.
    * <p>
    * It is important that the wrench describes the wrench of the control frame especially if only part
    * of the angular torque is to be controlled.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredWrench the desired end-effector wrench with respect to the base and expressed in
    *           the control frame.
    * @throws ReferenceFrameMismatchException if the {@code desiredWrench} is not setup as follows:
    *            {@code bodyFrame = endEffector.getBodyFixedFrame()},
    *            {@code baseFrame = base.getBodyFixedFrame()}, {@code expressedInFrame = controlFrame}.
    */
   public void setWrench(ReferenceFrame controlFrame, WrenchReadOnly desiredWrench)
   {
      desiredWrench.getBodyFrame().checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      desiredWrench.getReferenceFrame().checkReferenceFrameMatch(controlFrame);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      desiredAngularTorque.set(desiredWrench.getAngularPart());
      desiredLinearForce.set(desiredWrench.getLinearPart());
   }

   /**
    * Sets the desired wrench to submit for the optimization.
    * <p>
    * The {@code desiredAngularTorque} has to define the desired angular torque of the end-effector
    * with respect to the base. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The {@code desiredLinearForce} has to defined the desired linear force of the origin of the
    * {@code controlFrame}. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredAngularTorque the desired angular torque of the end-effector with respect to the
    *           base. Not modified.
    * @param desiredLinearForce the desired linear force of the origin of the control frame with
    *           respect to the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredAngularTorque} or
    *            {@code desiredLinearForce} is not expressed in control frame.
    */
   public void setWrench(ReferenceFrame controlFrame, FrameVector3D desiredAngularTorque, FrameVector3D desiredLinearForce)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularTorque);
      controlFrame.checkReferenceFrameMatch(desiredLinearForce);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
      this.desiredAngularTorque.set(desiredAngularTorque);
      this.desiredLinearForce.set(desiredLinearForce);
   }

   /**
    * Sets the desired angular torque to submit for the optimization and also set the linear part to
    * zero.
    * <p>
    * The {@code desiredAngularTorque} has to define the desired angular torque of the end-effector
    * with respect to the base. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}. </p*
    *
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredAngularTorque the desired angular torque of the end-effector with respect to the
    *           base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredAngularTorque} is not expressed in
    *            control frame.
    */
   public void setAngularTorque(ReferenceFrame controlFrame, FrameVector3D desiredAngularTorque)
   {
      controlFrame.checkReferenceFrameMatch(desiredAngularTorque);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      this.desiredAngularTorque.set(desiredAngularTorque);
      desiredLinearForce.setToZero();
   }

   /**
    * Sets the desired linear force to submit for the optimization and also set the angular part to
    * zero.
    * <p>
    * The {@code desiredLinearForce} has to defined the desired linear force of the origin of the
    * {@code controlFrame}. It has to be expressed in {@code controlFrame}.
    * </p>
    * <p>
    * The given {@code controlFrame} should be located at the point of interest and has to be attached
    * to the end-effector. For instance, when controlling a foot, the {@code controlFrame} should be
    * located somewhere on the sole of the foot.
    * </p>
    * <p>
    * If no particular location on the end-effector is to controlled, then simply provide
    * {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredLinearForce the desired linear force of the origin of the control frame with
    *           respect to the base. Not modified.
    * @throws ReferenceFrameMismatchException if {@code desiredLinearForce} is not expressed in control
    *            frame.
    */
   public void setLinearForce(ReferenceFrame controlFrame, FrameVector3D desiredLinearForce)
   {
      controlFrame.checkReferenceFrameMatch(desiredLinearForce);

      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());

      this.desiredLinearForce.set(desiredLinearForce);
      desiredAngularTorque.setToZero();
   }

   /**
    * Sets the selection matrix to be used for the next control tick to the 6-by-6 identity matrix.
    * <p>
    * This specifies that the 6 degrees of freedom of the end-effector are to be controlled.
    * </p>
    */
   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.resetSelection();
   }

   /**
    * Convenience method that sets up the selection matrix such that only the linear part of this
    * command will be considered in the control.
    */
   public void setSelectionMatrixForLinearControl()
   {
      selectionMatrix.setToLinearSelectionOnly();
   }

   /**
    * Convenience method that sets up the selection matrix by disabling the angular part of this
    * command and applying the given selection matrix to the linear part.
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
    * </p>
    * 
    * @param linearSelectionMatrix the selection matrix to apply to the linear part of this command.
    *           Not modified.
    */
   public void setSelectionMatrixForLinearControl(SelectionMatrix3D linearSelectionMatrix)
   {
      selectionMatrix.clearSelection();
      selectionMatrix.setLinearPart(linearSelectionMatrix);
   }

   /**
    * Convenience method that sets up the selection matrix such that only the angular part of this
    * command will be considered in the control.
    */
   public void setSelectionMatrixForAngularControl()
   {
      selectionMatrix.setToAngularSelectionOnly();
   }

   /**
    * Convenience method that sets up the selection matrix by disabling the linear part of this command
    * and applying the given selection matrix to the angular part.
    * <p>
    * If the selection frame is not set, i.e. equal to {@code null}, it is assumed that the selection
    * frame is equal to the control frame.
    * </p>
    * 
    * @param angularSelectionMatrix the selection matrix to apply to the angular part of this command.
    *           Not modified.
    */
   public void setSelectionMatrixForAngularControl(SelectionMatrix3D angularSelectionMatrix)
   {
      selectionMatrix.clearSelection();
      selectionMatrix.setAngularPart(angularSelectionMatrix);
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
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   public Vector3DBasics getDesiredLinearForce()
   {
      return desiredLinearForce;
   }

   public Vector3DBasics getDesiredAngularTorque()
   {
      return desiredAngularTorque;
   }

   /**
    * Packs the control frame and desired wrench held in this command.
    * <p>
    * The first argument {@code controlFrameToPack} is required to properly express the
    * {@code desiredWrenchToPack}. Indeed the desired wrench has to be expressed in the control frame.
    * </p>
    * 
    * @param controlFrameToPack the frame of interest for controlling the end-effector. Modified.
    * @param desiredWrenchToPack the desired wrench of the end-effector with respect to the base,
    *           expressed in the control frame. Modified.
    */
   public void getDesiredWrench(PoseReferenceFrame controlFrameToPack, Wrench desiredWrenchToPack)
   {
      getControlFrame(controlFrameToPack);
      desiredWrenchToPack.setToZero(endEffector.getBodyFixedFrame(), controlFrameToPack);
      desiredWrenchToPack.getLinearPart().set(desiredLinearForce);
      desiredWrenchToPack.getAngularPart().set(desiredAngularTorque);
   }

   /**
    * Packs the value of the desired wrench of the end-effector with respect to the base, expressed in
    * the control frame.
    * <p>
    * The control frame can be obtained via {@link #getControlFrame(PoseReferenceFrame)}.
    * </p>
    * 
    * @param desiredWrenchToPack the 6-by-1 matrix in which the value of the desired wrench is stored.
    *           The given matrix is reshaped to ensure proper size. Modified.
    */
   public void getDesiredWrench(DenseMatrix64F desiredWrenchToPack)
   {
      desiredWrenchToPack.reshape(6, 1);
      desiredAngularTorque.get(0, desiredWrenchToPack);
      desiredLinearForce.get(3, desiredWrenchToPack);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredEffort(DenseMatrix64F desiredWrenchToPack)
   {
      getDesiredWrench(desiredWrenchToPack);
   }

   public FramePose3DBasics getControlFramePose()
   {
      return controlFramePose;
   }

   /**
    * Updates the given {@code PoseReferenceFrame} to match the control frame to use with this command.
    * <p>
    * The control frame is assumed to be attached to the end-effector.
    * </p>
    * <p>
    * The wrench that this command holds onto is expressed in the control frame.
    * </p>
    * 
    * @param controlFrameToPack the {@code PoseReferenceFrame} used to clone the control frame.
    *           Modified.
    */
   @Override
   public void getControlFrame(PoseReferenceFrame controlFrameToPack)
   {
      controlFramePose.changeFrame(controlFrameToPack.getParent());
      controlFrameToPack.setPoseAndUpdate(controlFramePose);
      controlFramePose.changeFrame(endEffector.getBodyFixedFrame());
   }

   /**
    * Packs the pose of the control frame expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePoseToPack the pose of the control frame. Modified.
    */
   public void getControlFramePoseIncludingFrame(FramePose3D controlFramePoseToPack)
   {
      controlFramePoseToPack.setIncludingFrame(controlFramePose);
   }

   /**
    * Packs the position and orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param positionToPack the position of the {@code controlFrame}'s origin. Modified.
    * @param orientationToPack the orientation of the {@code controlFrame}. Modified.
    */
   public void getControlFramePoseIncludingFrame(FramePoint3D positionToPack, FrameQuaternion orientationToPack)
   {
      positionToPack.setIncludingFrame(controlFramePose.getPosition());
      orientationToPack.setIncludingFrame(controlFramePose.getOrientation());
   }

   /**
    * Gets the 6-by-6 selection matrix expressed in the given {@code destinationFrame} to use with this
    * command.
    * 
    * @param destinationFrame the reference frame in which the selection matrix should be expressed in.
    * @param selectionMatrixToPack the dense-matrix in which the selection matrix of this command is
    *           stored in. Modified.
    */
   @Override
   public void getSelectionMatrix(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrix.getCompactSelectionMatrixInFrame(destinationFrame, selectionMatrixToPack);
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   /**
    * Packs the value of the selection matrix carried by this command into the given
    * {@code selectionMatrixToPack}.
    * 
    * @param selectionMatrixToPack the selection matrix to pack.
    */
   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.set(selectionMatrix);
   }

   /**
    * Gets the reference to the base of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be used to control the end-effector.
    * </p>
    * 
    * @return the rigid-body located right before the first joint to be used for controlling the
    *         end-effector.
    */
   @Override
   public RigidBodyBasics getBase()
   {
      return base;
   }

   /**
    * Gets the reference to the end-effector of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints that
    * can be used to control the end-effector.
    * </p>
    * 
    * @return the rigid-body to be controlled.
    */
   @Override
   public RigidBodyBasics getEndEffector()
   {
      return endEffector;
   }

   /**
    * {@inheritDoc}
    * 
    * @return {@link ControllerCoreCommandType#TASKSPACE}.
    */
   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.VIRTUAL_WRENCH;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof VirtualWrenchCommand)
      {
         VirtualWrenchCommand other = (VirtualWrenchCommand) object;

         if (!controlFramePose.equals(other.controlFramePose))
            return false;
         if (!desiredLinearForce.equals(other.desiredLinearForce))
            return false;
         if (!desiredAngularTorque.equals(other.desiredAngularTorque))
            return false;
         if (!selectionMatrix.equals(other.selectionMatrix))
            return false;
         if (base != other.base)
            return false;
         if (endEffector != other.endEffector)
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
      return getClass().getSimpleName() + ": base = " + base + ", endEffector = " + endEffector + ", linear = " + desiredLinearForce + ", angular = "
            + desiredAngularTorque;
   }
}
