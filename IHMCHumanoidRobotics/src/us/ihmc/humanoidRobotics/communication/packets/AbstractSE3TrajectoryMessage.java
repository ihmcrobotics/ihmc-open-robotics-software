package us.ihmc.humanoidRobotics.communication.packets;

import static us.ihmc.humanoidRobotics.communication.packets.FrameBasedMessage.*;

import java.util.Random;

import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosIgnoredField;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public abstract class AbstractSE3TrajectoryMessage<T extends AbstractSE3TrajectoryMessage<T>> extends QueueableMessage<T>
      implements Transformable, FrameBasedMessage
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public SE3TrajectoryPointMessage[] taskspaceTrajectoryPoints;
   @RosIgnoredField
   public SelectionMatrix3DMessage angularSelectionMatrix;
   @RosIgnoredField
   public SelectionMatrix3DMessage linearSelectionMatrix;

   @RosExportedField(documentation = "The ID of the reference frame to execute the trajectory in")
   public long trajectoryReferenceFrameId;

   @RosExportedField(documentation = "The Id of the reference frame defining which frame the taskspaceTrajectoryPoints are expressed in")
   public long dataReferenceFrameId;

   @RosExportedField(documentation = "Flag that tells the controller whether the use of a custom control frame is requested.")
   public boolean useCustomControlFrame = false;

   @RosExportedField(documentation = "Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.")
   public QuaternionBasedTransform controlFramePose;

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
      for (int i = 0; i < randomNumberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(random);
      }
      trajectoryReferenceFrameId = ReferenceFrame.getWorldFrame().getNameBasedHashCode();
      dataReferenceFrameId = ReferenceFrame.getWorldFrame().getNameBasedHashCode();
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
      trajectoryReferenceFrameId = se3TrajectoryMessage.getTrajectoryReferenceFrameId();
      dataReferenceFrameId = se3TrajectoryMessage.getDataReferenceFrameId();

   }

   public AbstractSE3TrajectoryMessage(double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation, long dataFrameId,
                                       long trajectoryReferenceFrameId)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      Vector3D zeroLinearVelocity = new Vector3D();
      Vector3D zeroAngularVelocity = new Vector3D();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[] {
            new SE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
      this.trajectoryReferenceFrameId = trajectoryReferenceFrameId;
      this.dataReferenceFrameId = dataFrameId;
   }

   public AbstractSE3TrajectoryMessage(double trajectoryTime, Point3D desiredPosition, Quaternion desiredOrientation, ReferenceFrame dataFrame,
                                       ReferenceFrame trajectoryReferenceFrame)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      Vector3D zeroLinearVelocity = new Vector3D();
      Vector3D zeroAngularVelocity = new Vector3D();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[] {
            new SE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
      this.trajectoryReferenceFrameId = trajectoryReferenceFrame.getNameBasedHashCode();
      this.dataReferenceFrameId = dataFrame.getNameBasedHashCode();
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
      trajectoryReferenceFrameId = other.getTrajectoryReferenceFrameId();
      dataReferenceFrameId = other.getDataReferenceFrameId();
   }

   public void getTrajectoryPoints(FrameSE3TrajectoryPointList trajectoryPointListToPack)
   {
      checkIfTrajectoryFrameIdsMatch(this.dataReferenceFrameId, trajectoryPointListToPack.getReferenceFrame());
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
    * 
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when
    *           the trajectory starts.
    * @param position define the desired 3D position to be reached at this trajectory point. It is
    *           expressed in world frame.
    * @param orientation define the desired 3D orientation to be reached at this trajectory point.
    *           It is expressed in world frame.
    * @param linearVelocity define the desired 3D linear velocity to be reached at this trajectory
    *           point. It is expressed in world frame.
    * @param angularVelocity define the desired 3D angular velocity to be reached at this trajectory
    *           point. It is expressed in world frame.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3D position, Quaternion orientation, Vector3D linearVelocity,
                                        Vector3D angularVelocity, ReferenceFrame expressedInReferenceFrame)
   {
      checkIfTrajectoryFrameIdsMatch(this.dataReferenceFrameId, expressedInReferenceFrame);
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints[trajectoryPointIndex] = new SE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity);
   }

   /**
    * Create a trajectory point.
    * 
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when
    *           the trajectory starts.
    * @param position define the desired 3D position to be reached at this trajectory point. It is
    *           expressed in world frame.
    * @param orientation define the desired 3D orientation to be reached at this trajectory point.
    *           It is expressed in world frame.
    * @param linearVelocity define the desired 3D linear velocity to be reached at this trajectory
    *           point. It is expressed in world frame.
    * @param angularVelocity define the desired 3D angular velocity to be reached at this trajectory
    *           point. It is expressed in world frame.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3D position, Quaternion orientation, Vector3D linearVelocity,
                                        Vector3D angularVelocity, long expressedInReferenceFrameId)
   {
      checkIfFrameIdsMatch(this.dataReferenceFrameId, expressedInReferenceFrameId);
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
    * Sets the selection matrix to use for executing this message.
    * <p>
    * The selection matrix is used to determinate which degree of freedom of the end-effector should
    * be controlled. When it is NOT provided, the controller will assume that all the degrees of
    * freedom of the end-effector should be controlled.
    * </p>
    * <p>
    * The selection frames coming along with the given selection matrix are used to determine to
    * what reference frame the selected axes are referring to. For instance, if only the hand height
    * in world should be controlled on the linear z component of the selection matrix should be
    * selected and the reference frame should be world frame. When no reference frame is provided
    * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
    * frame if not defined otherwise.
    * </p>
    * 
    * @param selectionMatrix the selection matrix to use when executing this trajectory message. Not
    *           modified.
    */
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix6D)
   {
      if (angularSelectionMatrix == null)
         angularSelectionMatrix = new SelectionMatrix3DMessage(selectionMatrix6D.getAngularPart());
      else
         angularSelectionMatrix.set(selectionMatrix6D.getAngularPart());

      if (linearSelectionMatrix == null)
         linearSelectionMatrix = new SelectionMatrix3DMessage(selectionMatrix6D.getLinearPart());
      else
         linearSelectionMatrix.set(selectionMatrix6D.getLinearPart());
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
      return angularSelectionMatrix != null && linearSelectionMatrix != null;
   }

   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.resetSelection();
      if (angularSelectionMatrix != null)
         angularSelectionMatrix.getSelectionMatrix(selectionMatrixToPack.getAngularPart());
      if (linearSelectionMatrix != null)
         linearSelectionMatrix.getSelectionMatrix(selectionMatrixToPack.getLinearPart());
   }

   /**
    * Returns the unique ID referring to the selection frame to use with the angular part of the
    * selection matrix of this message.
    * <p>
    * If this message does not have a angular selection matrix, this method returns
    * {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    * 
    * @return the selection frame ID for the angular part of the selection matrix.
    */
   public long getAngularSelectionFrameId()
   {
      if (angularSelectionMatrix != null)
         return angularSelectionMatrix.getSelectionFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
   }

   /**
    * Returns the unique ID referring to the selection frame to use with the linear part of the
    * selection matrix of this message.
    * <p>
    * If this message does not have a linear selection matrix, this method returns
    * {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    * 
    * @return the selection frame ID for the linear part of the selection matrix.
    */
   public long getLinearSelectionFrameId()
   {
      if (linearSelectionMatrix != null)
         return linearSelectionMatrix.getSelectionFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
   }

   private void rangeCheck(int trajectoryPointIndex)
   {
      if (trajectoryPointIndex >= getNumberOfTrajectoryPoints() || trajectoryPointIndex < 0)
         throw new IndexOutOfBoundsException("Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: "
               + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {

      if (dataReferenceFrameId != other.getDataReferenceFrameId())
      {
         return false;
      }

      if (trajectoryReferenceFrameId != other.getTrajectoryReferenceFrameId())
      {
         return false;
      }

      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         return false;

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!taskspaceTrajectoryPoints[i].epsilonEquals(other.taskspaceTrajectoryPoints[i], epsilon))
            return false;
      }

      return super.epsilonEquals(other, epsilon);
   }

   public long getTrajectoryReferenceFrameId()
   {
      return trajectoryReferenceFrameId;
   }

   public void setTrajectoryReferenceFrameId(long trajectoryReferenceFrameId)
   {
      this.trajectoryReferenceFrameId = trajectoryReferenceFrameId;
   }

   public void setTrajectoryReferenceFrameId(ReferenceFrame trajectoryReferenceFrame)
   {
      trajectoryReferenceFrameId = trajectoryReferenceFrame.getNameBasedHashCode();
   }

   public long getDataReferenceFrameId()
   {
      return dataReferenceFrameId;
   }

   public void setDataReferenceFrameId(long expressedInReferenceFrameId)
   {
      this.dataReferenceFrameId = expressedInReferenceFrameId;
   }

   public void setDataReferenceFrameId(ReferenceFrame expressedInReferenceFrame)
   {
      this.dataReferenceFrameId = expressedInReferenceFrame.getNameBasedHashCode();
   }

   public void setUseCustomControlFrame(boolean useCustomControlFrame)
   {
      this.useCustomControlFrame = useCustomControlFrame;
   }

   public void setControlFramePosition(Point3D controlFramePosition)
   {
      if (controlFramePose == null)
         controlFramePose = new QuaternionBasedTransform();
      controlFramePose.setTranslation(controlFramePosition);
   }

   public void setControlFrameOrientation(Quaternion controlFrameOrientation)
   {
      if (controlFramePose == null)
         controlFramePose = new QuaternionBasedTransform();
      controlFramePose.setRotation(controlFrameOrientation);
   }

   public boolean useCustomControlFrame()
   {
      return useCustomControlFrame;
   }

   public void getControlFramePose(RigidBodyTransform controlFrameTransformToPack)
   {
      if (controlFramePose == null)
         controlFrameTransformToPack.setToNaN();
      else
         controlFrameTransformToPack.set(controlFramePose);
   }
}
