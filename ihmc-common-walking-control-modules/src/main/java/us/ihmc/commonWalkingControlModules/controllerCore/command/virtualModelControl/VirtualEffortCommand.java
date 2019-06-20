package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public interface VirtualEffortCommand<T extends VirtualEffortCommand<T>> extends VirtualModelControlCommand<T>
{
   /**
    * Updates the given {@code PoseReferenceFrame} to match the control frame to use with this
    * command.
    * <p>
    * The control frame is assumed to be attached to the end-effector.
    * </p>
    * <p>
    * The linear force that this command holds onto is expressed in the control frame.
    * </p>
    *
    * @param controlFrameToPack the {@code PoseReferenceFrame} used to clone the control frame.
    *           Modified.
    */
   void getControlFrame(PoseReferenceFrame controlFrameToPack);

   /**
    * Packs the value of the desired effort of the end-effector with respect to the
    * base, expressed in the control frame.
    * <p>
    * This can be a force, torque, or wrench. This results in a 3, 3, and 6 dimensional vector.
    * </p>
    * <p>
    * The control frame can be obtained via {@link #getControlFrame(PoseReferenceFrame)}.
    * </p>
    *
    * @param desiredEffortToPack the N-by-1 matrix in which the value of the desired
    *           effort is stored. The given matrix is reshaped to ensure proper size.
    *           Modified.
    */
   void getDesiredEffort(DenseMatrix64F desiredEffortToPack);

   /**
    * Gets the reference to the base of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints
    * that can be used to control the end-effector.
    * </p>
    *
    * @return the rigid-body located right before the first joint to be used for controlling the
    *         end-effector.
    */
   RigidBodyBasics getBase();

   /**
    * Gets the reference to the end-effector of this command.
    * <p>
    * The joint path going from the {@code base} to the {@code endEffector} specifies the joints
    * that can be used to control the end-effector.
    * </p>
    *
    * @return the rigid-body to be controlled.
    */
   RigidBodyBasics getEndEffector();

   /**
    * Gets the N-by-6 selection matrix expressed in the given {@code destinationFrame} to use with
    * this command.
    *
    * @param destinationFrame the reference frame in which the selection matrix should be expressed
    *           in.
    * @param selectionMatrixToPack the dense-matrix in which the selection matrix of this command is
    *           stored in. Modified.
    */
   void getSelectionMatrix(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack);
}
