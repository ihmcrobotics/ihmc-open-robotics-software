package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosIgnoredField;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.Transformable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class AbstractSE3TrajectoryMessage<T extends AbstractSE3TrajectoryMessage<T>> extends TrackablePacket<T>
      implements TransformableDataObject<T>, Transformable
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public SE3TrajectoryPointMessage[] taskspaceTrajectoryPoints;
   @RosExportedField(documentation = "When OVERRIDE is chosen:"
         + "\n - The time of the first trajectory point can be zero, in which case the controller will start directly at the first trajectory point."
         + " Otherwise the controller will prepend a first trajectory point at the current desired position." + "\n When QUEUE is chosen:"
         + "\n - The message must carry the ID of the message it should be queued to."
         + "\n - The very first message of a list of queued messages has to be an OVERRIDE message."
         + "\n - The trajectory point times are relative to the the last trajectory point time of the previous message."
         + "\n - The controller will queue the joint trajectory messages as a per joint basis." + " The first trajectory point has to be greater than zero.")
   public ExecutionMode executionMode = ExecutionMode.OVERRIDE;
   @RosExportedField(documentation = "Only needed when using QUEUE mode, it refers to the message Id to which this message should be queued to."
         + " It is used by the controller to ensure that no message has been lost on the way."
         + " If a message appears to be missing (previousMessageId different from the last message ID received by the controller), the motion is aborted."
         + " If previousMessageId == 0, the controller will not check for the ID of the last received message.")
   public long previousMessageId = INVALID_MESSAGE_ID;
   @RosIgnoredField
   public float[] selectionMatrixDiagonal;

   public AbstractSE3TrajectoryMessage()
   {
   }

   public AbstractSE3TrajectoryMessage(Random random)
   {
      int randomNumberOfPoints = random.nextInt(16) + 1;
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[randomNumberOfPoints];
      for(int i = 0; i < randomNumberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(random);
      }

      executionMode = RandomTools.generateRandomEnum(random, ExecutionMode.class);
   }

   public AbstractSE3TrajectoryMessage(T se3TrajectoryMessage)
   {
      int numberOfPoints = se3TrajectoryMessage.getNumberOfTrajectoryPoints();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(se3TrajectoryMessage.taskspaceTrajectoryPoints[i]);
      }
      executionMode = se3TrajectoryMessage.executionMode;
      previousMessageId = se3TrajectoryMessage.previousMessageId;
   }

   public AbstractSE3TrajectoryMessage(double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      Vector3d zeroLinearVelocity = new Vector3d();
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[] {
            new SE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
   }

   public AbstractSE3TrajectoryMessage(int numberOfTrajectoryPoints)
   {
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[numberOfTrajectoryPoints];
   }

   public void set(T other)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         throw new RuntimeException("Must the same number of waypoints.");
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(other.taskspaceTrajectoryPoints[i]);
      executionMode = other.executionMode;
      previousMessageId = other.previousMessageId;
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
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity,
         Vector3d angularVelocity)
   {
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints[trajectoryPointIndex] = new SE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i].applyTransform(transform);
   }

   /**
    * Set how the controller should consume this message:
    * <li> {@link ExecutionMode#OVERRIDE}: this message will override any previous message, including canceling any active execution of a message.
    * <li> {@link ExecutionMode#QUEUE}: this message is queued and will be executed once all the previous messages are done.
    * @param executionMode
    * @param previousMessageId when queuing, one needs to provide the ID of the message this message should be queued to.
    */
   public void setExecutionMode(ExecutionMode executionMode, long previousMessageId)
   {
      this.executionMode = executionMode;
      this.previousMessageId = previousMessageId;
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

   public ExecutionMode getExecutionMode()
   {
      return executionMode;
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

   public long getPreviousMessageId()
   {
      return previousMessageId;
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
      if (executionMode != other.executionMode)
         return false;
      if (executionMode == ExecutionMode.OVERRIDE && previousMessageId != other.previousMessageId)
         return false;

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!taskspaceTrajectoryPoints[i].epsilonEquals(other.taskspaceTrajectoryPoints[i], epsilon))
            return false;
      }

      return true;
   }
}
