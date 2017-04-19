package us.ihmc.communication.packets;

import static us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage.*;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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
public class KinematicsToolboxCenterOfMassMessage extends TrackablePacket<KinematicsToolboxCenterOfMassMessage>
{
   /**
    * This is the desired center of mass position. The data is assumed to be expressed in world
    * frame.
    */
   public Point3D32 desiredPositionInWorld;

   /**
    * Array of 3 booleans used to create a selection matrix on the solver side:<br>
    * <code>boolean[] selectionMatrix = {controlX, controlY, controlZ};</code>
    * <p>
    * The three booleans define which axis to control the position of in the world frame.
    * </p>
    */
   public boolean[] selectionMatrix;
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

   /**
    * Creates a new center of mass message.
    * <p>
    * The new message is ready to be sent, but it can be further adjusted to provide more details.
    * For example, the priority of the task can be changed by changing the weight of this message, a
    * custom control frame can be specified.
    * </p>
    * 
    * @param desiredPosition the position that center of mass should reach. The data is assumed to
    *           be expressed in world frame. Not modified.
    */
   public KinematicsToolboxCenterOfMassMessage(Point3DReadOnly desiredPosition)
   {
      setDesiredPosition(desiredPosition);
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
   public void setDesiredPosition(FramePoint desiredPosition)
   {
      desiredPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setDesiredPosition(desiredPosition.getPoint());
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
    * Ensures that the array for the selection matrix is initialized.
    * <p>
    * If the array has to be created, it is initialized with {@code true}s.
    * </p>
    */
   private void initializeSelectionMatrix()
   {
      if (selectionMatrix == null)
      {
         selectionMatrix = new boolean[3];
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
    * Sets the selection matrix by setting individually which axis is to be controlled.
    * 
    * @param enableAxesControl array of 3 booleans specifying whether each axis is to be controlled,
    *           in order: linearX, linearY, linearZ. Not modified.
    * @throws RuntimeException if the given array has length different than 3.
    */
   public void setSelectionMatrix(boolean[] enableAxesControl)
   {
      if (enableAxesControl.length != 3)
         throw new RuntimeException("Unexpected size. Expected 3, was: " + enableAxesControl.length);
      initializeSelectionMatrix();
      for (int i = 0; i < 3; i++)
         selectionMatrix[i] = enableAxesControl[i];
   }

   /**
    * Sets the selection matrix by setting individually which axis is to be controlled.
    * 
    * @param enableX whether to control the x-axis.
    * @param enableY whether to control the y-axis.
    * @param enableZ whether to control the z-axis.
    */
   public void setSelectionMatrix(boolean enableX, boolean enableY, boolean enableZ)
   {
      initializeSelectionMatrix();
      selectionMatrix[0] = enableX;
      selectionMatrix[1] = enableY;
      selectionMatrix[2] = enableZ;
   }

   public void getDesiredPosition(FramePoint desiredPositionToPack)
   {
      desiredPositionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), desiredPositionInWorld);
   }

   public void getSelectionMatrix(DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrixToPack.reshape(3, 6);
      for (int i = 0; i < 3; i++)
         selectionMatrixToPack.set(i, i, 1.0);

      if (selectionMatrix != null)
      {
         for (int i = 2; i >= 0; i--)
         {
            if (!selectionMatrix[i])
               MatrixTools.removeRow(selectionMatrixToPack, i);
         }
      }
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
}
