package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosIgnoredField;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractSE3TrajectoryMessage<T extends AbstractSE3TrajectoryMessage<T>> extends QueueableMessage<T> implements Transformable
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public SE3TrajectoryPointMessage[] taskspaceTrajectoryPoints;
   @RosIgnoredField
   public float[] selectionMatrixDiagonal;

   public AbstractSE3TrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public AbstractSE3TrajectoryMessage(Random random)
   {
      super(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      int randomNumberOfPoints = random.nextInt(16) + 1;
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[randomNumberOfPoints];
      for(int i = 0; i < randomNumberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(random);
      }
   }

   public AbstractSE3TrajectoryMessage(T se3TrajectoryMessage)
   {
      int numberOfPoints = se3TrajectoryMessage.getNumberOfTrajectoryPoints();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(se3TrajectoryMessage.taskspaceTrajectoryPoints[i]);
      }

      setExecutionMode(se3TrajectoryMessage.getExecutionMode(), se3TrajectoryMessage.getPreviousMessageId());
      setUniqueId(se3TrajectoryMessage.getUniqueId());
      setDestination(se3TrajectoryMessage.getDestination());
   }

   public AbstractSE3TrajectoryMessage(double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      Vector3D zeroLinearVelocity = new Vector3D();
      Vector3D zeroAngularVelocity = new Vector3D();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[] {
            new SE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
   }

   public AbstractSE3TrajectoryMessage(int numberOfTrajectoryPoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[numberOfTrajectoryPoints];
   }

   public void set(T other)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         throw new RuntimeException("Must the same number of waypoints.");
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(other.taskspaceTrajectoryPoints[i]);
      setExecutionMode(other.getExecutionMode(), other.getPreviousMessageId());
   }

   public void getTrajectoryPoints(FrameSE3TrajectoryPointList trajectoryPointListToPack)
   {
      trajectoryPointListToPack.clear(ReferenceFrame.getWorldFrame());

      SE3TrajectoryPointMessage[] trajectoryPointMessages = getTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.length;

      for (int i = 0; i < numberOfPoints; i++)
      {
         SE3TrajectoryPointMessage se3TrajectoryPointMessage = trajectoryPointMessages[i];
         trajectoryPointListToPack.addTrajectoryPoint(se3TrajectoryPointMessage.time, se3TrajectoryPointMessage.position, se3TrajectoryPointMessage.orientation,
               se3TrajectoryPointMessage.linearVelocity, se3TrajectoryPointMessage.angularVelocity);
      }
   }

   /**
    * Create a trajectory point.
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.
    * @param orientation define the desired 3D orientation to be reached at this trajectory point. It is expressed in world frame.
    * @param linearVelocity define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.
    * @param angularVelocity define the desired 3D angular velocity to be reached at this trajectory point. It is expressed in world frame.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3D position, Quaternion orientation, Vector3D linearVelocity,
         Vector3D angularVelocity)
   {
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints[trajectoryPointIndex] = new SE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i].applyTransform(transform);
   }

   /**
    * The selectionMatrix needs to be 6x6.
    * @param selectionMatrix
    */
   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrixDiagonal == null)
         selectionMatrixDiagonal = new float[6];

      DenseMatrix64F inner = new DenseMatrix64F(selectionMatrix.getNumCols(), selectionMatrix.getNumCols());
      CommonOps.multInner(selectionMatrix, inner);

      for (int i = 0; i < inner.getNumRows(); i++)
         selectionMatrixDiagonal[i] = (float) inner.get(i, i);
   }

   public final int getNumberOfTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints.length;
   }

   public final SE3TrajectoryPointMessage[] getTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   public final SE3TrajectoryPointMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      rangeCheck(trajectoryPointIndex);
      return taskspaceTrajectoryPoints[trajectoryPointIndex];
   }

   public final SE3TrajectoryPointMessage getLastTrajectoryPoint()
   {
      return taskspaceTrajectoryPoints[taskspaceTrajectoryPoints.length - 1];
   }

   public final double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().time;
   }

   public boolean hasSelectionMatrix()
   {
      return selectionMatrixDiagonal != null;
   }

   public void getSelectionMatrix(DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrixToPack.reshape(6, 6);
      if (selectionMatrixDiagonal == null)
      {
         CommonOps.setIdentity(selectionMatrixToPack);
      }
      else
      {
         selectionMatrixToPack.zero();
         for (int i = 0; i < 6; i++)
            selectionMatrixToPack.set(i, i, selectionMatrixDiagonal[i]);
         MatrixTools.removeZeroRows(selectionMatrixToPack, 1.0e-5);
      }
   }

   private void rangeCheck(int trajectoryPointIndex)
   {
      if (trajectoryPointIndex >= getNumberOfTrajectoryPoints() || trajectoryPointIndex < 0)
         throw new IndexOutOfBoundsException(
               "Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: " + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         return false;

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!taskspaceTrajectoryPoints[i].epsilonEquals(other.taskspaceTrajectoryPoints[i], epsilon))
            return false;
      }

      return super.epsilonEquals(other, epsilon);
   }
}
