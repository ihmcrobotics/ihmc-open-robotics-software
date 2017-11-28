package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.packets.WeightMatrix3DMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosIgnoredField;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public abstract class AbstractEuclideanTrajectoryMessage<T extends AbstractEuclideanTrajectoryMessage<T>> extends QueueableMessage<T>
      implements Transformable, FrameBasedMessage
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory.")
   public EuclideanTrajectoryPointMessage[] taskspaceTrajectoryPoints;

   /** the selection matrix for each axis **/
   @RosIgnoredField
   public SelectionMatrix3DMessage linearSelectionMatrix = new SelectionMatrix3DMessage();

   @RosExportedField(documentation = "Frame information for this message.")
   public FrameInformation frameInformation = new FrameInformation();

   /** the weight matrix for each axis **/
   @RosIgnoredField
   public WeightMatrix3DMessage linearWeightMatrix = new WeightMatrix3DMessage();

   @RosExportedField(documentation = "Flag that tells the controller whether the use of a custom control frame is requested.")
   public boolean useCustomControlFrame = false;

   @RosExportedField(documentation = "Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.")
   public QuaternionBasedTransform controlFramePose = new QuaternionBasedTransform();

   public AbstractEuclideanTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public AbstractEuclideanTrajectoryMessage(Random random)
   {
      super(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);

      int randomNumberOfPoints = random.nextInt(16) + 1;
      taskspaceTrajectoryPoints = new EuclideanTrajectoryPointMessage[randomNumberOfPoints];
      for (int i = 0; i < randomNumberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new EuclideanTrajectoryPointMessage(random);
      }

      useCustomControlFrame = random.nextBoolean();
      controlFramePose.set(RandomGeometry.nextQuaternion(random), RandomGeometry.nextVector3D(random));
   }

   public AbstractEuclideanTrajectoryMessage(T trajectoryMessage)
   {
      int numberOfPoints = trajectoryMessage.getNumberOfTrajectoryPoints();
      taskspaceTrajectoryPoints = new EuclideanTrajectoryPointMessage[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new EuclideanTrajectoryPointMessage(trajectoryMessage.taskspaceTrajectoryPoints[i]);
      }

      setExecutionMode(trajectoryMessage.getExecutionMode(), trajectoryMessage.getPreviousMessageId());
      setExecutionDelayTime(trajectoryMessage.getExecutionDelayTime());
      setUniqueId(trajectoryMessage.getUniqueId());
      setDestination(trajectoryMessage.getDestination());
      linearSelectionMatrix.set(trajectoryMessage.linearSelectionMatrix);
      frameInformation.set(trajectoryMessage);
      linearWeightMatrix.set(trajectoryMessage.linearWeightMatrix);
      useCustomControlFrame = trajectoryMessage.useCustomControlFrame;
      controlFramePose.set(trajectoryMessage.controlFramePose);
   }

   /**
    * set a single point
    * @param trajectoryTime the duration of the trajectory
    * @param desiredPosition the desired end position
    * @param trajectoryReferenceFrameId the frame id the trajectory will be executed in
    */
   public AbstractEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, long trajectoryReferenceFrameId)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      Vector3D zeroLinearVelocity = new Vector3D();
      taskspaceTrajectoryPoints = new EuclideanTrajectoryPointMessage[] {
            new EuclideanTrajectoryPointMessage(trajectoryTime, desiredPosition, zeroLinearVelocity)};
      frameInformation.setTrajectoryReferenceFrameId(trajectoryReferenceFrameId);
   }

   /**
    * set a single point
    * @param trajectoryTime the duration of the trajectory
    * @param desiredPosition the desired end position
    * @param trajectoryReferenceFrame the frame the trajectory will be executed in
    */
   public AbstractEuclideanTrajectoryMessage(double trajectoryTime, Point3DReadOnly desiredPosition, ReferenceFrame trajectoryReferenceFrame)
   {
      this(trajectoryTime, desiredPosition, trajectoryReferenceFrame.getNameBasedHashCode());
   }

   /**
    * creates a new empty message with a trajectory point list the size of numberOfTrajectoryPoints
    * @param numberOfTrajectoryPoints number of trajectory points in this message
    */
   public AbstractEuclideanTrajectoryMessage(int numberOfTrajectoryPoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      taskspaceTrajectoryPoints = new EuclideanTrajectoryPointMessage[numberOfTrajectoryPoints];
   }

   /**
    * set this message to the have the same contents of the other message
    * @param other the other message
    */
   public void set(T other)
   {
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         throw new RuntimeException("Must the same number of waypoints.");
      int numberOfPoints = other.getNumberOfTrajectoryPoints();
      taskspaceTrajectoryPoints = new EuclideanTrajectoryPointMessage[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         taskspaceTrajectoryPoints[i] = new EuclideanTrajectoryPointMessage(other.taskspaceTrajectoryPoints[i]);
      }

      setExecutionMode(other.getExecutionMode(), other.getPreviousMessageId());
      setExecutionDelayTime(other.getExecutionDelayTime());
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());
      linearSelectionMatrix.set(other.linearSelectionMatrix);
      frameInformation.set(other);
      linearWeightMatrix.set(other.linearWeightMatrix);
      useCustomControlFrame = other.useCustomControlFrame;
      controlFramePose.set(other.controlFramePose);
   }

   /**
    * Get the trajectory points from this message
    * @param trajectoryPointListToPack
    */
   public void getTrajectoryPoints(FrameEuclideanTrajectoryPointList trajectoryPointListToPack)
   {
      FrameInformation.checkIfDataFrameIdsMatch(frameInformation, trajectoryPointListToPack.getReferenceFrame());
      EuclideanTrajectoryPointMessage[] trajectoryPointMessages = getTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.length;

      for (int i = 0; i < numberOfPoints; i++)
      {
         EuclideanTrajectoryPointMessage euclideanTrajectoryPointMessage = trajectoryPointMessages[i];
         trajectoryPointListToPack.addTrajectoryPoint(euclideanTrajectoryPointMessage.time, euclideanTrajectoryPointMessage.position,
               euclideanTrajectoryPointMessage.linearVelocity);
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
    * @param linearVelocity define the desired 3D linear velocity to be reached at this trajectory
    *           point. It is expressed in world frame.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity,
         ReferenceFrame expressedInReferenceFrame)
   {
      FrameInformation.checkIfDataFrameIdsMatch(frameInformation, expressedInReferenceFrame);
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints[trajectoryPointIndex] = new EuclideanTrajectoryPointMessage(time, position, linearVelocity);
   }

   /**
    * Create a trajectory point.
    *
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when
    *           the trajectory starts.
    * @param position define the desired 3D position to be reached at this trajectory point. It is
    *           expressed in world frame.
    * @param linearVelocity define the desired 3D linear velocity to be reached at this trajectory
    *           point. It is expressed in world frame.
    */
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity,
         long expressedInReferenceFrameId)
   {
      FrameInformation.checkIfDataFrameIdsMatch(frameInformation, expressedInReferenceFrameId);
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints[trajectoryPointIndex] = new EuclideanTrajectoryPointMessage(time, position, linearVelocity);
   }

   /**
    * transform all the points
    */
   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i].applyTransform(transform);
   }

   /**
    * transform all the points
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i].applyInverseTransform(transform);
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
    * @param selectionMatrix3D the selection matrix to use when executing this trajectory message. Not
    *           modified.
    */
   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix3D)
   {
      if (linearSelectionMatrix == null)
         linearSelectionMatrix = new SelectionMatrix3DMessage(selectionMatrix3D);
      else
         linearSelectionMatrix.set(selectionMatrix3D);
   }

   /**
    * Sets the weight matrix to use for executing this message.
    * <p>
    * The weight matrix is used to set the qp weights for the controlled degrees of freedom of the end-effector.
    * When it is not provided, or when the weights are set to Double.NaN, the controller will use the default QP Weights
    * set for each axis.
    * </p>
    * <p>
    * The selection frame coming along with the given weight matrix is used to determine to what
    * reference frame the weights are referring to.
    * </p>
    *
    * @param weightMatrix the selection matrix to use when executing this trajectory message. parameter is not
    *           modified.
    */
   public void setWeightMatrix(WeightMatrix3D weightMatrix)
   {
      if (linearWeightMatrix == null)
         linearWeightMatrix = new WeightMatrix3DMessage(weightMatrix);
      else
         linearWeightMatrix.set(weightMatrix);
   }

   public final int getNumberOfTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints.length;
   }

   /**
    * Returns the internal mutable list of points, modifying this list changes the internal message
    * @return
    */
   public final EuclideanTrajectoryPointMessage[] getTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   public final EuclideanTrajectoryPointMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      rangeCheck(trajectoryPointIndex);
      return taskspaceTrajectoryPoints[trajectoryPointIndex];
   }

   public final EuclideanTrajectoryPointMessage getLastTrajectoryPoint()
   {
      return taskspaceTrajectoryPoints[taskspaceTrajectoryPoints.length - 1];
   }

   public final double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().time;
   }

   public boolean hasSelectionMatrix()
   {
      return linearSelectionMatrix != null;
   }

   public void getSelectionMatrix(SelectionMatrix3D selectionMatrixToPack)
   {
      selectionMatrixToPack.resetSelection();
      if (linearSelectionMatrix != null)
         linearSelectionMatrix.getSelectionMatrix(selectionMatrixToPack);
   }

   public boolean hasWeightMatrix()
   {
      return linearWeightMatrix != null;
   }

   public void getWeightMatrix(WeightMatrix3D weightMatrixToPack)
   {
      weightMatrixToPack.clear();
      if (linearWeightMatrix != null)
         linearWeightMatrix.getWeightMatrix(weightMatrixToPack);
   }

   @Override
   public FrameInformation getFrameInformation()
   {
      if (frameInformation == null)
         frameInformation = new FrameInformation();
      return frameInformation;
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

   /**
    * Returns the unique ID referring to the linear weight frame to use with the weight matrix of
    * this message.
    * <p>
    * If this message does not have a linear weight matrix, this method returns
    * {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    *
    * @return the weight frame ID for the angular weight matrix.
    */
   public long getLinearWeightMatrixFrameId()
   {
      if (linearWeightMatrix != null)
         return linearWeightMatrix.getWeightFrameId();
      else
         return NameBasedHashCodeTools.NULL_HASHCODE;
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
      if (!frameInformation.epsilonEquals(other.frameInformation, epsilon))
      {
         return false;
      }

      if (linearSelectionMatrix == null && other.linearSelectionMatrix != null)
      {
         return false;
      }

      if (linearSelectionMatrix != null && !linearSelectionMatrix.epsilonEquals(other.linearSelectionMatrix, epsilon))
      {
         return false;
      }

      if (linearWeightMatrix == null && other.linearWeightMatrix != null)
      {
         return false;
      }

      if (linearWeightMatrix != null && !linearWeightMatrix.epsilonEquals(other.linearWeightMatrix, epsilon))
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

   public void setUseCustomControlFrame(boolean useCustomControlFrame)
   {
      this.useCustomControlFrame = useCustomControlFrame;
   }

   public void setControlFramePosition(Point3DReadOnly controlFramePosition)
   {
      if (controlFramePose == null)
         controlFramePose = new QuaternionBasedTransform();
      controlFramePose.setTranslation(controlFramePosition);
   }

   public void setControlFrameOrientation(QuaternionReadOnly controlFrameOrientation)
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