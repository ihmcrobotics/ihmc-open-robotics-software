package us.ihmc.communication.packets;

import static us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage.nullEqualsAndEpsilonEquals;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

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
   public Point3D32 desiredPositionInWorld;

   // TODO add doc
   public SelectionMatrix3DMessage selectionMatrix;
   /**
    * Array of 3 floats used to define the priority of this task on the solver side:<br>
    * <code>float[] weights = {weightX, weightY, weightZ};</code>
    * <p>
    * The three floats refer to the priority of controlling the position along each axis of the
    * center of mass frame.
    * </p>
    */
   public float[] weights;

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
      if (other.weights != null)
         weights = Arrays.copyOf(other.weights, other.weights.length);
      setPacketInformation(other);
   }

   /**
    * Sets the desired position that the center of mass should reach. The data is assumed to be
    * expressed in world frame.
    * 
    * @param desiredPosition the position the center of mass should reach. Not modified.
    */
   public void setDesiredPosition(Point3DReadOnly desiredPosition)
   {
      if (desiredPositionInWorld == null)
         desiredPositionInWorld = new Point3D32(desiredPosition);
      else
         desiredPositionInWorld.set(desiredPosition);
   }

   /**
    * Sets the desired position that the center of mass should reach. The data is assumed to be
    * expressed in world frame.
    * 
    * @param desiredPosition the position the center of mass should reach. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in world frame.
    */
   public void setDesiredPosition(FramePoint3D desiredPosition)
   {
      desiredPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredPosition(desiredPosition);
   }

   /** Ensures that the array for the weights is initialized. */
   private void initializeWeight()
   {
      if (weights == null)
         weights = new float[3];
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
      for (int i = 0; i < 3; i++)
         weights[i] = (float) weight;
   }

   /**
    * Enables the control of all the degrees of freedom of the center of mass.
    */
   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix = new SelectionMatrix3DMessage();
   }

   /**
    * Sets the selection matrix to use for executing this message.
    * <p>
    * The selection matrix is used to determinate which degree of freedom of the center of mass
    * should be controlled. When it is NOT provided, the controller will assume that all the degrees
    * of freedom should be controlled.
    * </p>
    * <p>
    * The selection frame coming along with the given selection matrix is used to determine to what
    * reference frame the selected axes are referring to. For instance, if only the hand height in
    * world should be controlled on the linear z component of the selection matrix should be
    * selected and the reference frame should world frame. When no reference frame is provided with
    * the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed frame
    * if not defined otherwise.
    * </p>
    * 
    * @param selectionMatrix the selection matrix to use when executing this trajectory message. Not
    *           modified.
    */
   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      if (this.selectionMatrix == null)
         this.selectionMatrix = MessageTools.createSelectionMatrix3DMessage(selectionMatrix);
      else
         this.selectionMatrix.set(selectionMatrix);
   }

   public void getDesiredPosition(FramePoint3D desiredPositionToPack)
   {
      desiredPositionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), desiredPositionInWorld);
   }

   public void getSelectionMatrix(SelectionMatrix3D selectionMatrixToPack)
   {
      selectionMatrixToPack.clearSelection();
      if (selectionMatrix != null)
         selectionMatrix.getSelectionMatrix(selectionMatrixToPack);
   }

   /**
    * Returns the unique ID referring to the selection frame to use with the selection matrix of
    * this message.
    * <p>
    * If this message does not have a selection matrix, this method returns
    * {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    * 
    * @return the selection frame ID for the selection matrix.
    */
   public long getSelectionFrameId()
   {
      if (selectionMatrix != null)
         return selectionMatrix.getSelectionFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
   }

   public void getWeightVector(DenseMatrix64F weightVectorToPack)
   {
      weightVectorToPack.reshape(3, 1);
      if (weights == null)
      {
         weightVectorToPack.zero();
      }
      else
      {
         for (int i = 0; i < 3; i++)
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
   public boolean epsilonEquals(KinematicsToolboxCenterOfMassMessage other, double epsilon)
   {
      if (!nullEqualsAndEpsilonEquals(desiredPositionInWorld, desiredPositionInWorld, epsilon))
         return false;

      // TODO Add the selection matrix in there

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
}
