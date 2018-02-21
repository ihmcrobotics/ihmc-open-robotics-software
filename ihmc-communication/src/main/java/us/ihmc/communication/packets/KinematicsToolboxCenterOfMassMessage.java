package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D32;

/**
 * {@link KinematicsToolboxCenterOfMassMessage} is part of the API of the
 * {@code KinematicsToolboxController}.
 * <p>
 * It holds all the information needed for the {@code KinematicsToolboxController} to constrain the
 * center of mass to a desired location.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxCenterOfMassMessage extends Packet<KinematicsToolboxCenterOfMassMessage>
{
   /**
    * This is the desired center of mass position. The data is assumed to be expressed in world
    * frame.
    */
   public Point3D32 desiredPositionInWorld = new Point3D32();
   /**
    * The selection matrix is used to determinate which degree of freedom of the center of mass
    * should be controlled. When it is NOT provided, the controller will assume that all the degrees
    * of freedom should be controlled.
    * <p>
    * The selection frame coming along with the given selection matrix is used to determine to what
    * reference frame the selected axes are referring to. For instance, if only the hand height in
    * world should be controlled on the linear z component of the selection matrix should be
    * selected and the reference frame should world frame. When no reference frame is provided with
    * the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed frame
    * if not defined otherwise.
    * </p>
    * 
    */
   public SelectionMatrix3DMessage selectionMatrix = new SelectionMatrix3DMessage();
   /**
    * Array of 3 floats used to define the priority of this task on the solver side:<br>
    * <code>float[] weights = {weightX, weightY, weightZ};</code>
    * <p>
    * The three floats refer to the priority of controlling the position along each axis of the
    * center of mass frame.
    * </p>
    */
   public WeightMatrix3DMessage weights = new WeightMatrix3DMessage();

   public KinematicsToolboxCenterOfMassMessage()
   {
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(KinematicsToolboxCenterOfMassMessage other)
   {
      desiredPositionInWorld = new Point3D32(other.desiredPositionInWorld);
      if (other.selectionMatrix != null)
      {
         selectionMatrix = new SelectionMatrix3DMessage();
         selectionMatrix.set(other.selectionMatrix);
      }
      weights.set(other.weights);
      setPacketInformation(other);
   }

   public SelectionMatrix3DMessage getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public WeightMatrix3DMessage getWeights()
   {
      return weights;
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
   public boolean epsilonEquals(KinematicsToolboxCenterOfMassMessage other, double epsilon)
   {
      if (!desiredPositionInWorld.epsilonEquals(desiredPositionInWorld, epsilon))
         return false;

      if (weights == null && other.weights == null)
         return true;
      if (weights == null && other.weights != null)
         return false;
      if (weights != null && other.weights == null)
         return false;
      if (!weights.epsilonEquals(other.weights, epsilon))
         return false;
      if (!selectionMatrix.epsilonEquals(other.selectionMatrix, epsilon))
         return false;
      return true;
   }
}
