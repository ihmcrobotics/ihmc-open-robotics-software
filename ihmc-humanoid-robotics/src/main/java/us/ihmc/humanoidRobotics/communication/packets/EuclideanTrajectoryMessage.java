package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.packets.WeightMatrix3DMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.idl.PreallocatedList;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/euclidean_trajectory")
public final class EuclideanTrajectoryMessage extends Packet<EuclideanTrajectoryMessage>
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory.")
   public PreallocatedList<EuclideanTrajectoryPointMessage> taskspaceTrajectoryPoints = new PreallocatedList<>(EuclideanTrajectoryPointMessage.class, EuclideanTrajectoryPointMessage::new, 100);

   @RosExportedField(documentation = "The selection matrix for each axis.")
   public SelectionMatrix3DMessage selectionMatrix = new SelectionMatrix3DMessage();

   @RosExportedField(documentation = "Frame information for this message.")
   public FrameInformation frameInformation = new FrameInformation();

   @RosExportedField(documentation = "The weight matrix for each axis.")
   public WeightMatrix3DMessage weightMatrix = new WeightMatrix3DMessage();

   @RosExportedField(documentation = "Flag that tells the controller whether the use of a custom control frame is requested.")
   public boolean useCustomControlFrame = false;

   @RosExportedField(documentation = "Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.")
   public Pose3D controlFramePose = new Pose3D();

   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   public EuclideanTrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public EuclideanTrajectoryMessage(EuclideanTrajectoryMessage other)
   {
      MessageTools.copyData(other.taskspaceTrajectoryPoints, taskspaceTrajectoryPoints);
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());
      frameInformation.set(other.getFrameInformation());
      if (other.selectionMatrix != null)
         selectionMatrix.set(other.selectionMatrix);
      if (other.weightMatrix != null)
         weightMatrix.set(other.weightMatrix);
      useCustomControlFrame = other.useCustomControlFrame;
      controlFramePose.set(other.controlFramePose);
      if (other.queueingProperties != null)
         queueingProperties.set(other.queueingProperties);
   }

   /**
    * set this message to the have the same contents of the other message
    * 
    * @param other the other message
    */
   @Override
   public void set(EuclideanTrajectoryMessage other)
   {
      MessageTools.copyData(other.taskspaceTrajectoryPoints, taskspaceTrajectoryPoints);
      selectionMatrix.set(other.selectionMatrix);
      frameInformation.set(other.getFrameInformation());
      weightMatrix.set(other.weightMatrix);
      useCustomControlFrame = other.useCustomControlFrame;
      controlFramePose.set(other.controlFramePose);
      queueingProperties.set(other.queueingProperties);

      setPacketInformation(other);
   }

   /**
    * Get the trajectory points from this message
    * 
    * @param trajectoryPointListToPack
    */
   public void getTrajectoryPoints(FrameEuclideanTrajectoryPointList trajectoryPointListToPack)
   {
      FrameInformation.checkIfDataFrameIdsMatch(frameInformation, trajectoryPointListToPack.getReferenceFrame());
      PreallocatedList<EuclideanTrajectoryPointMessage> trajectoryPointMessages = getTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.size();

      for (int i = 0; i < numberOfPoints; i++)
      {
         EuclideanTrajectoryPointMessage euclideanTrajectoryPointMessage = trajectoryPointMessages.get(i);
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
      taskspaceTrajectoryPoints.get(trajectoryPointIndex).set(HumanoidMessageTools.createEuclideanTrajectoryPointMessage(time, position, linearVelocity));
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
      taskspaceTrajectoryPoints.get(trajectoryPointIndex).set(HumanoidMessageTools.createEuclideanTrajectoryPointMessage(time, position, linearVelocity));
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
    * @param selectionMatrix3D the selection matrix to use when executing this trajectory message.
    *           Not modified.
    */
   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix3D)
   {
      if (selectionMatrix == null)
         selectionMatrix = MessageTools.createSelectionMatrix3DMessage(selectionMatrix3D);
      else
         selectionMatrix.set(selectionMatrix3D);
   }

   /**
    * Sets the weight matrix to use for executing this message.
    * <p>
    * The weight matrix is used to set the qp weights for the controlled degrees of freedom of the
    * end-effector. When it is not provided, or when the weights are set to Double.NaN, the
    * controller will use the default QP Weights set for each axis.
    * </p>
    * <p>
    * The selection frame coming along with the given weight matrix is used to determine to what
    * reference frame the weights are referring to.
    * </p>
    *
    * @param weightMatrix3D the selection matrix to use when executing this trajectory message.
    *           parameter is not modified.
    */
   public void setWeightMatrix(WeightMatrix3D weightMatrix3D)
   {
      if (weightMatrix == null)
         weightMatrix = MessageTools.createWeightMatrix3DMessage(weightMatrix3D);
      else
         weightMatrix.set(weightMatrix3D);
   }

   public final int getNumberOfTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints.size();
   }

   /**
    * Returns the internal mutable list of points, modifying this list changes the internal message
    * 
    * @return
    */
   public final PreallocatedList<EuclideanTrajectoryPointMessage> getTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   public final EuclideanTrajectoryPointMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      rangeCheck(trajectoryPointIndex);
      return taskspaceTrajectoryPoints.get(trajectoryPointIndex);
   }

   public final EuclideanTrajectoryPointMessage getLastTrajectoryPoint()
   {
      return taskspaceTrajectoryPoints.get(taskspaceTrajectoryPoints.size() - 1);
   }

   public final double getTrajectoryTime()
   {
      return getLastTrajectoryPoint().time;
   }

   public boolean hasSelectionMatrix()
   {
      return selectionMatrix != null;
   }

   public void getSelectionMatrix(SelectionMatrix3D selectionMatrixToPack)
   {
      selectionMatrixToPack.resetSelection();
      if (selectionMatrix != null)
         selectionMatrix.getSelectionMatrix(selectionMatrixToPack);
   }

   public boolean hasWeightMatrix()
   {
      return weightMatrix != null;
   }

   public void getWeightMatrix(WeightMatrix3D weightMatrixToPack)
   {
      weightMatrixToPack.clear();
      if (weightMatrix != null)
         weightMatrix.getWeightMatrix(weightMatrixToPack);
   }

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
      if (selectionMatrix != null)
         return selectionMatrix.getSelectionFrameId();
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
      if (weightMatrix != null)
         return weightMatrix.getWeightFrameId();
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
   public boolean epsilonEquals(EuclideanTrajectoryMessage other, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(other.queueingProperties, epsilon))
      {
         return false;
      }

      if (!frameInformation.epsilonEquals(other.frameInformation, epsilon))
      {
         return false;
      }

      if (selectionMatrix == null ^ other.selectionMatrix == null)
      {
         return false;
      }

      if (selectionMatrix != null && !selectionMatrix.epsilonEquals(other.selectionMatrix, epsilon))
      {
         return false;
      }

      if (weightMatrix == null ^ other.weightMatrix == null)
      {
         return false;
      }

      if (weightMatrix != null && !weightMatrix.epsilonEquals(other.weightMatrix, epsilon))
      {
         return false;
      }

      if (!MessageTools.epsilonEquals(taskspaceTrajectoryPoints, other.taskspaceTrajectoryPoints, epsilon))
         return false;

      return true;
   }

   public void setUseCustomControlFrame(boolean useCustomControlFrame)
   {
      this.useCustomControlFrame = useCustomControlFrame;
   }

   public void setControlFramePosition(Point3DReadOnly controlFramePosition)
   {
      if (controlFramePose == null)
         controlFramePose = new Pose3D();
      controlFramePose.setPosition(controlFramePosition);
   }

   public void setControlFrameOrientation(QuaternionReadOnly controlFrameOrientation)
   {
      if (controlFramePose == null)
         controlFramePose = new Pose3D();
      controlFramePose.setOrientation(controlFrameOrientation);
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
         controlFramePose.get(controlFrameTransformToPack);
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }
}