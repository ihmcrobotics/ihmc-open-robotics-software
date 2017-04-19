package us.ihmc.communication.packets;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * {@link KinematicsToolboxRigidBodyMessage} is part of the API {@code KinematicsToolboxController}.
 * <p>
 * It holds all the information needed for the {@code KinematicsToolboxController} to constrain the
 * given end-effector to certain number of task-space constraints, i.e. desired
 * position/orientation.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxRigidBodyMessage extends TrackablePacket<KinematicsToolboxRigidBodyMessage>
{
   /**
    * This is the unique hash code of the end-effector to be solved for. It used on the solver side
    * to retrieve the desired end-effector to be controlled.
    */
   public long endEffectorNameBasedHashCode;
   /**
    * This is the desired position of the control frame's origin. If the control frame has not been
    * defined, it represents the desired position of {@code endEffector.getBodyFixedFrame()}'s
    * origin. The data is assumed to be expressed in world frame.
    */
   public Point3D32 desiredPositionInWorld;
   /**
    * This is the desired orientation of the control frame. If the control frame has not been
    * defined, it represents the desired orientation of {@code endEffector.getBodyFixedFrame()}. The
    * data is assumed to be expressed in world frame.
    */
   public Quaternion32 desiredOrientationInWorld;
   /**
    * This is the position of the control frame's origin expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Point3D32 controlFramePositionInEndEffector;
   /**
    * This is the orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Quaternion32 controlFrameOrientationInEndEffector;
   /**
    * Array of 6 booleans used to create a selection matrix on the solver side:<br>
    * <code>boolean[] selectionMatrix = {controlAngularX, controlAngularY, controlAngularZ, controlLinearX, controlLinearY, controlLinearZ};</code>
    * <ul>
    * <li>The three booleans <code>(controlAngularX, controlAngularY, controlAngularZ)</code> define
    * which axis to control the orientation of in the control frame. If {@code controlFrame} has not
    * been defined, it is instead in {@code endEffector.getBodyFixedFrame()}.
    * <li>The three booleans <code>(controlLinearX, controlLinearY, controlLinearZ)</code> define
    * which axis to control the position of in the control frame. If {@code controlFrame} has not
    * been defined, it is instead in {@code endEffector.getBodyFixedFrame()}.
    * </ul>
    */
   public boolean[] selectionMatrix;
   /**
    * Array of 6 floats used to define the priority of this task on the solver side:<br>
    * <code>float[] weights = {weightAngularX, weightAngularY, weightAngularZ, weightLinearX, weightLinearY, weightLinearZ};</code>
    * <ul>
    * <li>The three floats <code>(weightAngularX, weightAngularY, weightAngularZ)</code> refer to
    * the priority of controlling the rotation around each axis of the control frame. If
    * {@code controlFrame} has not been defined, it is instead
    * {@code endEffector.getBodyFixedFrame()}.
    * <li>The three floats <code>(weightLinearX, weightLinearY, weightLinearZ)</code> refer to the
    * priority of controlling the position along each axis of the control frame. If
    * {@code controlFrame} has not been defined, it is instead
    * {@code endEffector.getBodyFixedFrame()}.
    * </ul>
    */
   public float[] weights;

   /**
    * Do not use this constructor, it is needed only for efficient serialization/deserialization.
    */
   public KinematicsToolboxRigidBodyMessage()
   {
      // empty constructor for deserialization
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * Before the message can be sent to the solver, you will need to provide at least a desired
    * orientation and/or desired position.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    */
   public KinematicsToolboxRigidBodyMessage(RigidBody endEffector)
   {
      endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for linear control only.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public KinematicsToolboxRigidBodyMessage(RigidBody endEffector, Point3DReadOnly desiredPosition)
   {
      endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      setDesiredPosition(desiredPosition);
      setSelectionMatrixForLinearControl();
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * <p>
    * Note that this constructor also sets up the selection matrix for angular control only.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public KinematicsToolboxRigidBodyMessage(RigidBody endEffector, QuaternionReadOnly desiredOrientation)
   {
      endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      setDesiredOrientation(desiredOrientation);
      setSelectionMatrixForAngularControl();
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public KinematicsToolboxRigidBodyMessage(RigidBody endEffector, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation)
   {
      endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      setDesiredPose(desiredPosition, desiredOrientation);
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Creates a new rigid-body message for the given end-effector.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param endEffector the end-effector to solver for in the {@code KinematicsToolboxController}.
    * @param controlFrame specifies the location and orientation of interest for controlling the
    *           end-effector.
    * @param desiredPosition the position that {@code endEffector.getBodyFixedFrame()}'s origin
    *           should reach. The data is assumed to be expressed in world frame. Not modified.
    * @param desiredOrientation the orientation that {@code endEffector.getBodyFixedFrame()} should
    *           reach. The data is assumed to be expressed in world frame. Not modified.
    */
   public KinematicsToolboxRigidBodyMessage(RigidBody endEffector, ReferenceFrame controlFrame, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation)
   {
      endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      setDesiredPose(desiredPosition, desiredOrientation);
      RigidBodyTransform transformToBodyFixedFrame = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(transformToBodyFixedFrame, endEffector.getBodyFixedFrame());
      setControlFramePose(transformToBodyFixedFrame.getTranslationVector(), transformToBodyFixedFrame.getRotationMatrix());
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Sets the desired position that the control frame's origin should reach. By default the control
    * frame is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be
    * expressed in world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    */
   public void setDesiredPosition(Point3DReadOnly desiredPosition)
   {
      if (desiredPositionInWorld == null)
         desiredPositionInWorld = new Point3D32(desiredPosition);
      else
         desiredPositionInWorld.set(desiredPosition);
   }

   /**
    * Sets the desired position that the control frame's origin should reach. By default the control
    * frame is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be
    * expressed in world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in world frame.
    */
   public void setDesiredPosition(FramePoint desiredPosition)
   {
      desiredPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredPosition(desiredPosition.getPoint());
   }

   /**
    * Sets the desired orientation that the control frame should reach. By default the control frame
    * is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed
    * in world frame.
    * 
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    */
   public void setDesiredOrientation(QuaternionReadOnly desiredOrientation)
   {
      if (desiredOrientationInWorld == null)
         desiredOrientationInWorld = new Quaternion32(desiredOrientation);
      else
         desiredOrientationInWorld.set(desiredOrientation);
   }

   /**
    * Sets the desired orientation that the control frame should reach. By default the control frame
    * is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed
    * in world frame.
    * 
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in world frame.
    */
   public void setDesiredOrientation(FrameOrientation desiredOrientation)
   {
      desiredOrientation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredOrientation(desiredOrientation.getQuaternion());
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    */
   public void setDesiredPose(Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation)
   {
      setDesiredPosition(desiredPosition);
      setDesiredOrientation(desiredOrientation);
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPose the pose the control frame should reach. Not modified.
    */
   public void setDesiredPose(Pose pose)
   {
      setDesiredPosition(pose.getPosition());
      setDesiredOrientation(pose.getOrientation());
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in world
    *            frame.
    */
   public void setDesiredPose(FramePoint desiredPosition, FrameOrientation desiredOrientation)
   {
      setDesiredPosition(desiredPosition);
      setDesiredOrientation(desiredOrientation);
   }

   /**
    * Sets the desired pose that the control frame should reach. By default the control frame is
    * coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed in
    * world frame.
    * 
    * @param desiredPose the pose the control frame should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in world frame.
    */
   public void setDesiredPose(FramePose desiredPose)
   {
      desiredPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredPose(desiredPose.getGeometryObject());
   }

   /** Ensures that the array for the weights is initialized. */
   private void initializeWeight()
   {
      if (weights == null)
         weights = new float[6];
   }

   /**
    * Sets the weight to use for this task.
    * <p>
    * The weight relates to the priority of a task relative to the other active tasks. A higher
    * weight refers to a higher priority.
    * </p>
    * 
    * @param weight the weight value for this task.
    */
   public void setWeight(double weight)
   {
      initializeWeight();
      for (int i = 0; i < 6; i++)
         weights[i] = (float) weight;
   }

   /**
    * Sets the weight to use for this task with the angular and linear parts set independently.
    * <p>
    * The weight relates to the priority of a task relative to the other active tasks. A higher
    * weight refers to a higher priority.
    * </p>
    * 
    * @param angular the weight to use for the angular part of this task.
    * @param linear the weight to use for the linear part of this task.
    */
   public void setWeight(double angular, double linear)
   {
      initializeWeight();
      for (int i = 0; i < 3; i++)
      {
         weights[i] = (float) angular;
         weights[i + 3] = (float) linear;
      }
   }

   /**
    * Ensures that the array for the selection matrix is initialized.
    * <p>
    * If the array has to be created, it is initialized with {@code true}s.
    * </p>
    */
   private void initializeSelectionMatrix()
   {
      if (selectionMatrix == null)
      {
         selectionMatrix = new boolean[6];
         Arrays.fill(selectionMatrix, true);
      }
   }

   /**
    * Enables the control of all the degrees of freedom of the end-effector.
    */
   public void setSelectionMatrixToIdentity()
   {
      initializeSelectionMatrix();
      Arrays.fill(selectionMatrix, true);
   }

   /**
    * Enables the control for the translational degrees of freedom of the end-effector and disable
    * the rotational part.
    */
   public void setSelectionMatrixForLinearControl()
   {
      initializeSelectionMatrix();
      for (int i = 0; i < 3; i++)
      {
         selectionMatrix[i] = false;
         selectionMatrix[i + 3] = true;
      }
   }

   /**
    * Enables the control for the rotational degrees of freedom of the end-effector and disable the
    * translational part.
    */
   public void setSelectionMatrixForAngularControl()
   {
      initializeSelectionMatrix();
      for (int i = 0; i < 3; i++)
      {
         selectionMatrix[i] = true;
         selectionMatrix[i + 3] = false;
      }
   }

   /**
    * Sets the selection matrix by setting individually which axis is to be controlled.
    * 
    * @param enableAxesControl array of 6 booleans specifying whether each axis is to be controlled,
    *           in order: angularX, angularY, angularZ, linearX, linearY, linearZ. Not modified.
    * @throws RuntimeException if the given array has length different than 6.
    */
   public void setSelectionMatrix(boolean[] enableAxesControl)
   {
      if (enableAxesControl.length != 6)
         throw new RuntimeException("Unexpected size. Expected 6, was: " + enableAxesControl.length);
      initializeSelectionMatrix();
      for (int i = 0; i < 6; i++)
         selectionMatrix[i] = enableAxesControl[i];
   }

   /**
    * Specifies whether the rotation about the x-axis of the control frame should be controlled or
    * not.
    * <p>
    * By default the control frame is coincident to {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param enable {@code true} to control the degree of freedom of the end-effector, {@code false}
    *           otherwise.
    */
   public void enableAngularControlX(boolean enable)
   {
      initializeSelectionMatrix();
      selectionMatrix[0] = enable;
   }

   /**
    * Specifies whether the rotation about the y-axis of the control frame should be controlled or
    * not.
    * <p>
    * By default the control frame is coincident to {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param enable {@code true} to control the degree of freedom of the end-effector, {@code false}
    *           otherwise.
    */
   public void enableAngularControlY(boolean enable)
   {
      initializeSelectionMatrix();
      selectionMatrix[1] = enable;
   }

   /**
    * Specifies whether the rotation about the z-axis of the control frame should be controlled or
    * not.
    * <p>
    * By default the control frame is coincident to {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param enable {@code true} to control the degree of freedom of the end-effector, {@code false}
    *           otherwise.
    */
   public void enableAngularControlZ(boolean enable)
   {
      initializeSelectionMatrix();
      selectionMatrix[2] = enable;
   }

   /**
    * Specifies whether the position along the x-axis of the control frame should be controlled or
    * not.
    * <p>
    * By default the control frame is coincident to {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param enable {@code true} to control the degree of freedom of the end-effector, {@code false}
    *           otherwise.
    */
   public void enableLinearControlX(boolean enable)
   {
      initializeSelectionMatrix();
      selectionMatrix[3] = enable;
   }

   /**
    * Specifies whether the position along the y-axis of the control frame should be controlled or
    * not.
    * <p>
    * By default the control frame is coincident to {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param enable {@code true} to control the degree of freedom of the end-effector, {@code false}
    *           otherwise.
    */
   public void enableLinearControlY(boolean enable)
   {
      initializeSelectionMatrix();
      selectionMatrix[4] = enable;
   }

   /**
    * Specifies whether the position along the z-axis of the control frame should be controlled or
    * not.
    * <p>
    * By default the control frame is coincident to {@code endEffector.getBodyFixedFrame()}.
    * </p>
    * 
    * @param enable {@code true} to control the degree of freedom of the end-effector, {@code false}
    *           otherwise.
    */
   public void enableLinearControlZ(boolean enable)
   {
      initializeSelectionMatrix();
      selectionMatrix[5] = enable;
   }

   /**
    * Resets the control frame so it is coincident with {@code endEffector.getBodyFixedFrame()}.
    */
   public void resetControlFrame()
   {
      if (controlFramePositionInEndEffector != null)
         controlFramePositionInEndEffector.setToZero();
      if (controlFrameOrientationInEndEffector != null)
         controlFrameOrientationInEndEffector.setToZero();
   }

   /**
    * Specifies the position of the control frame's origin. The given point is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePosition(Tuple3DReadOnly controlFramePosition)
   {
      if (controlFramePositionInEndEffector == null)
         controlFramePositionInEndEffector = new Point3D32(controlFramePosition);
      else
         controlFramePositionInEndEffector.set(controlFramePosition);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientation(QuaternionReadOnly controlFrameOrientation)
   {
      if (controlFrameOrientationInEndEffector == null)
         controlFrameOrientationInEndEffector = new Quaternion32(controlFrameOrientation);
      else
         controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientation(RotationMatrixReadOnly controlFrameOrientation)
   {
      if (controlFrameOrientationInEndEffector == null)
         controlFrameOrientationInEndEffector = new Quaternion32(controlFrameOrientation);
      else
         controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given point and quaternion are assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Tuple3DReadOnly controlFramePosition, QuaternionReadOnly controlFrameOrientation)
   {
      setControlFramePosition(controlFramePosition);
      setControlFrameOrientation(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given point and quaternion are assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Tuple3DReadOnly controlFramePosition, RotationMatrixReadOnly controlFrameOrientation)
   {
      setControlFramePosition(controlFramePosition);
      setControlFrameOrientation(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given pose is assumed to be expressed in
    * {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePose the pose of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Pose controlFramePose)
   {
      setControlFramePosition(controlFramePose.getPosition());
      setControlFrameOrientation(controlFramePose.getOrientation());
   }

   public long getEndEffectorNameBasedHashCode()
   {
      return endEffectorNameBasedHashCode;
   }

   public void getDesiredPose(FramePose desiredPoseToPack)
   {
      desiredPoseToPack.setToZero(ReferenceFrame.getWorldFrame());

      if (desiredPositionInWorld != null)
         desiredPoseToPack.setPosition(desiredPositionInWorld);
      if (desiredOrientationInWorld != null)
         desiredPoseToPack.setOrientation(desiredOrientationInWorld);
   }

   public void getControlFramePose(RigidBody endEffector, FramePose controlFramePoseToPack)
   {
      ReferenceFrame referenceFrame = endEffector == null ? null : endEffector.getBodyFixedFrame();
      controlFramePoseToPack.setToZero(referenceFrame);

      if (controlFramePositionInEndEffector != null)
         controlFramePoseToPack.setPosition(controlFramePositionInEndEffector);
      if (controlFrameOrientationInEndEffector != null)
         controlFramePoseToPack.setOrientation(controlFrameOrientationInEndEffector);
   }

   public void getSelectionMatrix(DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrixToPack.reshape(6, 6);
      CommonOps.setIdentity(selectionMatrixToPack);

      if (selectionMatrix != null)
      {
         for (int i = 5; i >= 0; i--)
         {
            if (!selectionMatrix[i])
               MatrixTools.removeRow(selectionMatrixToPack, i);
         }
      }
   }

   public void getWeightVector(DenseMatrix64F weightVectorToPack)
   {
      weightVectorToPack.reshape(6, 1);
      if (weights == null)
      {
         weightVectorToPack.zero();
      }
      else
      {
         for (int i = 0; i < 6; i++)
            weightVectorToPack.set(i, 0, weights[i]);
      }
   }

   /**
    * Compares each field of this message against the other message and returns {@code true} if they
    * are equal to an {@code epsilon}.
    * <p>
    * Note that this method considers two fields to be equal if they are both {@code null}, and
    * considers two fields to be different if only one is equal to {@code null}.
    * </p>
    * 
    * @return {@code true} if the two messages are equal to an {@code epsilon}, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(KinematicsToolboxRigidBodyMessage other, double epsilon)
   {
      if (endEffectorNameBasedHashCode != other.endEffectorNameBasedHashCode)
         return false;
      if (!nullEqualsAndEpsilonEquals(desiredPositionInWorld, other.desiredPositionInWorld, epsilon))
         return false;
      if (!nullEqualsAndEpsilonEquals(desiredOrientationInWorld, other.desiredOrientationInWorld, epsilon))
         return false;
      if (!nullEqualsAndEpsilonEquals(controlFramePositionInEndEffector, other.controlFramePositionInEndEffector, epsilon))
         return false;
      if (!nullEqualsAndEpsilonEquals(controlFrameOrientationInEndEffector, other.controlFrameOrientationInEndEffector, epsilon))
         return false;
      if (!Arrays.equals(selectionMatrix, other.selectionMatrix))
         return false;

      if (weights == null && other.weights == null)
         return true;
      if (weights == null && other.weights != null)
         return false;
      if (weights != null && other.weights == null)
         return false;
      for (int i = 0; i < 6; i++)
      {
         if (!MathTools.epsilonEquals(weights[i], other.weights[i], epsilon))
            return false;
      }
      return true;
   }

   /**
    * Convenience method that first performs {@code null} tests before returning the result from
    * {@link EpsilonComparable#epsilonEquals(Object, double)}.
    * 
    * @param a the first object to compare. Not modified.
    * @param b the second object to compare. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the object are either {@code null} or not {@code null} and
    *         epsilon-equal, {@code false} if only one object is {@code null} or if both objects are
    *         not {@code null} but are not epsilon-equal.
    */
   static <T extends EpsilonComparable<T>> boolean nullEqualsAndEpsilonEquals(T a, T b, double epsilon)
   {
      if (a == null && b == null)
         return true;
      if (a == null && b != null)
         return false;
      if (a != null && b == null)
         return false;
      return a.epsilonEquals(b, epsilon);
   }
}
