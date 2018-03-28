package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * {@link KinematicsToolboxRigidBodyMessage} is part of the API of the
 * {@code KinematicsToolboxController}.
 * <p>
 * It holds all the information needed for the {@code KinematicsToolboxController} to constrain the
 * given end-effector to certain number of task-space constraints, i.e. desired
 * position/orientation.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxRigidBodyMessage extends Packet<KinematicsToolboxRigidBodyMessage>
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
   public Point3D32 desiredPositionInWorld = new Point3D32();
   /**
    * This is the desired orientation of the control frame. If the control frame has not been
    * defined, it represents the desired orientation of {@code endEffector.getBodyFixedFrame()}. The
    * data is assumed to be expressed in world frame.
    */
   public Quaternion32 desiredOrientationInWorld = new Quaternion32();
   /**
    * This is the position of the control frame's origin expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Point3D32 controlFramePositionInEndEffector = new Point3D32();
   /**
    * This is the orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Quaternion32 controlFrameOrientationInEndEffector = new Quaternion32();

   /**
    * The selection matrix is used to determinate which degree of freedom of the end-effector should
    * be controlled. When it is NOT provided, the controller will assume that all the degrees of
    * freedom of the end-effector should be controlled.
    * <p>
    * The selection frames coming along with the given selection matrix are used to determine to
    * what reference frame the selected axes are referring to. For instance, if only the hand height
    * in world should be controlled on the linear z component of the selection matrix should be
    * selected and the reference frame should be world frame. When no reference frame is provided
    * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
    * frame if not defined otherwise.
    * </p>
    */
   public SelectionMatrix3DMessage angularSelectionMatrix = new SelectionMatrix3DMessage();
   /**
    * The selection matrix is used to determinate which degree of freedom of the end-effector should
    * be controlled. When it is NOT provided, the controller will assume that all the degrees of
    * freedom of the end-effector should be controlled.
    * <p>
    * The selection frames coming along with the given selection matrix are used to determine to
    * what reference frame the selected axes are referring to. For instance, if only the hand height
    * in world should be controlled on the linear z component of the selection matrix should be
    * selected and the reference frame should be world frame. When no reference frame is provided
    * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
    * frame if not defined otherwise.
    * </p>
    */
   public SelectionMatrix3DMessage linearSelectionMatrix = new SelectionMatrix3DMessage();

   /**
    * Weight Matrix used to define the priority of controlling the rotation around each axis on the
    * solver side:<br>
    */
   public WeightMatrix3DMessage angularWeightMatrix = new WeightMatrix3DMessage();

   /**
    * Weight Matrix used to define the priority of controlling the translation of each axis on the
    * solver side:<br>
    */
   public WeightMatrix3DMessage linearWeightMatrix = new WeightMatrix3DMessage();

   /**
    * Do not use this constructor, it is needed only for efficient serialization/deserialization.
    */
   public KinematicsToolboxRigidBodyMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(KinematicsToolboxRigidBodyMessage other)
   {
      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredOrientationInWorld.set(other.desiredOrientationInWorld);
      controlFramePositionInEndEffector.set(other.controlFramePositionInEndEffector);
      controlFrameOrientationInEndEffector.set(other.controlFrameOrientationInEndEffector);
      angularSelectionMatrix.set(other.angularSelectionMatrix);
      linearSelectionMatrix.set(other.linearSelectionMatrix);
      angularWeightMatrix.set(other.angularWeightMatrix);
      linearWeightMatrix.set(other.linearWeightMatrix);
      setPacketInformation(other);
   }

   /**
    * Sets the desired position that the control frame's origin should reach. By default the control
    * frame is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be
    * expressed in world frame.
    * 
    * @param desiredPosition the position the control frame's origin should reach. Not modified.
    */
   public void setDesiredPositionInWorld(Point3DReadOnly desiredPosition)
   {
      desiredPositionInWorld.set(desiredPosition);
   }

   /**
    * Sets the desired orientation that the control frame should reach. By default the control frame
    * is coincident to {@code endEffector.getBodyFixedFrame()}. The data is assumed to be expressed
    * in world frame.
    * 
    * @param desiredOrientation the orientation the control frame should reach. Not modified.
    */
   public void setDesiredOrientationInWorld(QuaternionReadOnly desiredOrientation)
   {
      desiredOrientationInWorld.set(desiredOrientation);
   }

   /**
    * Specifies the position of the control frame's origin. The given point is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePositionInEndEffector(Tuple3DReadOnly controlFramePosition)
   {
      controlFramePositionInEndEffector.set(controlFramePosition);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientationInEndEffector(QuaternionReadOnly controlFrameOrientation)
   {
      controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   public long getEndEffectorNameBasedHashCode()
   {
      return endEffectorNameBasedHashCode;
   }

   public SelectionMatrix3DMessage getAngularSelectionMatrix()
   {
      return angularSelectionMatrix;
   }

   public SelectionMatrix3DMessage getLinearSelectionMatrix()
   {
      return linearSelectionMatrix;
   }

   public WeightMatrix3DMessage getAngularWeightMatrix()
   {
      return angularWeightMatrix;
   }

   public WeightMatrix3DMessage getLinearWeightMatrix()
   {
      return linearWeightMatrix;
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
      if (!desiredPositionInWorld.epsilonEquals(other.desiredPositionInWorld, epsilon))
         return false;
      if (!desiredOrientationInWorld.epsilonEquals(other.desiredOrientationInWorld, epsilon))
         return false;
      if (!controlFramePositionInEndEffector.epsilonEquals(other.controlFramePositionInEndEffector, epsilon))
         return false;
      if (!controlFrameOrientationInEndEffector.epsilonEquals(other.controlFrameOrientationInEndEffector, epsilon))
         return false;
      if (!linearWeightMatrix.epsilonEquals(linearWeightMatrix, epsilon))
         return false;
      if (!angularWeightMatrix.epsilonEquals(angularWeightMatrix, epsilon))
         return false;
      if (!linearSelectionMatrix.epsilonEquals(linearSelectionMatrix, epsilon))
         return false;
      if (!angularSelectionMatrix.epsilonEquals(angularSelectionMatrix, epsilon))
         return false;
      return true;
   }
}
