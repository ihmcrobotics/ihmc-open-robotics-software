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
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/se3_trajectory")
public final class SE3TrajectoryMessage extends Packet<SE3TrajectoryMessage>
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public PreallocatedList<SE3TrajectoryPointMessage> taskspaceTrajectoryPoints = new PreallocatedList<>(SE3TrajectoryPointMessage.class, SE3TrajectoryPointMessage::new, 100);
   @RosExportedField(documentation = "The selection matrix for each axis of the angular part.")
   public SelectionMatrix3DMessage angularSelectionMatrix;
   @RosExportedField(documentation = "The selection matrix for each axis of the linear part.")
   public SelectionMatrix3DMessage linearSelectionMatrix;

   @RosExportedField(documentation = "Frame information for this message.")
   public FrameInformation frameInformation = new FrameInformation();

   @RosExportedField(documentation = "The weight matrix for each axis of the angular part.")
   public WeightMatrix3DMessage angularWeightMatrix;
   @RosExportedField(documentation = "The weight matrix for each axis of the linear part.")
   public WeightMatrix3DMessage linearWeightMatrix;

   @RosExportedField(documentation = "Flag that tells the controller whether the use of a custom control frame is requested.")
   public boolean useCustomControlFrame = false;

   @RosExportedField(documentation = "Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.")
   public Pose3D controlFramePose;
   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   public SE3TrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public SE3TrajectoryMessage(SE3TrajectoryMessage other)
   {
      MessageTools.copyData(other.taskspaceTrajectoryPoints, taskspaceTrajectoryPoints);
      setUniqueId(other.getUniqueId());
      setDestination(other.getDestination());

      frameInformation.set(other.getFrameInformation());

      if (other.angularSelectionMatrix != null)
      {
         angularSelectionMatrix = new SelectionMatrix3DMessage(other.angularSelectionMatrix);
      }

      if (other.linearSelectionMatrix != null)
      {
         linearSelectionMatrix = new SelectionMatrix3DMessage(other.linearSelectionMatrix);
      }

      if (other.angularWeightMatrix != null)
      {
         angularWeightMatrix = new WeightMatrix3DMessage(other.angularWeightMatrix);
      }

      if (other.linearWeightMatrix != null)
      {
         linearWeightMatrix = new WeightMatrix3DMessage(other.linearWeightMatrix);
      }

      useCustomControlFrame = other.useCustomControlFrame;
      if (other.controlFramePose != null)
      {
         controlFramePose = new Pose3D(other.controlFramePose);
      }

      queueingProperties.set(other.queueingProperties);
   }

   @Override
   public void set(SE3TrajectoryMessage other)
   {
      MessageTools.copyData(other.taskspaceTrajectoryPoints, taskspaceTrajectoryPoints);
      frameInformation.set(other.getFrameInformation());
      queueingProperties.set(other.queueingProperties);
      setPacketInformation(other);
   }

   public void getTrajectoryPoints(FrameSE3TrajectoryPointList trajectoryPointListToPack)
   {
      FrameInformation.checkIfDataFrameIdsMatch(frameInformation, trajectoryPointListToPack.getReferenceFrame());
      PreallocatedList<SE3TrajectoryPointMessage> trajectoryPointMessages = getTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.size();

      for (int i = 0; i < numberOfPoints; i++)
      {
         SE3TrajectoryPointMessage se3TrajectoryPointMessage = trajectoryPointMessages.get(i);
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
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3DReadOnly position, QuaternionReadOnly orientation,
                                        Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity, ReferenceFrame expressedInReferenceFrame)
   {
      FrameInformation.checkIfDataFrameIdsMatch(frameInformation, expressedInReferenceFrame);
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints.get(trajectoryPointIndex).set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
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
   public final void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3DReadOnly position, QuaternionReadOnly orientation,
                                        Vector3DReadOnly linearVelocity, Vector3DReadOnly angularVelocity, long expressedInReferenceFrameId)
   {
      FrameInformation.checkIfDataFrameIdsMatch(frameInformation, expressedInReferenceFrameId);
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints.get(trajectoryPointIndex).set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
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
         angularSelectionMatrix = MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart());
      else
         angularSelectionMatrix.set(selectionMatrix6D.getAngularPart());

      if (linearSelectionMatrix == null)
         linearSelectionMatrix = MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getLinearPart());
      else
         linearSelectionMatrix.set(selectionMatrix6D.getLinearPart());
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
    * @param weightMatrix the selection matrix to use when executing this trajectory message.
    *           parameter is not modified.
    */
   public void setWeightMatrix(WeightMatrix6D weightMatrix)
   {
      if (angularWeightMatrix == null)
         angularWeightMatrix = MessageTools.createWeightMatrix3DMessage(weightMatrix.getAngularPart());
      else
         angularWeightMatrix.set(weightMatrix.getAngularPart());

      if (linearWeightMatrix == null)
         linearWeightMatrix = MessageTools.createWeightMatrix3DMessage(weightMatrix.getLinearPart());
      else
         linearWeightMatrix.set(weightMatrix.getLinearPart());
   }

   public final int getNumberOfTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints.size();
   }

   public final PreallocatedList<SE3TrajectoryPointMessage> getTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   public final SE3TrajectoryPointMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      rangeCheck(trajectoryPointIndex);
      return taskspaceTrajectoryPoints.get(trajectoryPointIndex);
   }

   public final SE3TrajectoryPointMessage getLastTrajectoryPoint()
   {
      return taskspaceTrajectoryPoints.get(taskspaceTrajectoryPoints.size() - 1);
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

   public boolean hasWeightMatrix()
   {
      return angularWeightMatrix != null && linearWeightMatrix != null;
   }

   public void getWeightMatrix(WeightMatrix6D weightMatrixToPack)
   {
      weightMatrixToPack.clear();
      if (angularWeightMatrix != null)
         angularWeightMatrix.getWeightMatrix(weightMatrixToPack.getAngularPart());
      if (linearWeightMatrix != null)
         linearWeightMatrix.getWeightMatrix(weightMatrixToPack.getLinearPart());
   }

   public FrameInformation getFrameInformation()
   {
      if (frameInformation == null)
         frameInformation = new FrameInformation();
      return frameInformation;
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

   /**
    * Returns the unique ID referring to the angular weight frame to use with the weight matrix of
    * this message.
    * <p>
    * If this message does not have an angular weight matrix, this method returns
    * {@link NameBasedHashCodeTools#NULL_HASHCODE}.
    * </p>
    *
    * @return the weight frame ID for the angular weight matrix.
    */
   public long getAngularWeightMatrixFrameId()
   {
      if (angularWeightMatrix != null)
         return angularWeightMatrix.getWeightFrameId();
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
         throw new IndexOutOfBoundsException("Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: "
               + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(SE3TrajectoryMessage other, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(other.queueingProperties, epsilon))
      {
         return false;
      }

      if (!frameInformation.epsilonEquals(other.frameInformation, epsilon))
      {
         return false;
      }

      if (linearSelectionMatrix == null ^ other.linearSelectionMatrix == null)
      {
         return false;
      }

      if (linearSelectionMatrix != null && !linearSelectionMatrix.epsilonEquals(other.linearSelectionMatrix, epsilon))
      {
         return false;
      }

      if (angularSelectionMatrix == null ^ other.angularSelectionMatrix == null)
      {
         return false;
      }

      if (angularSelectionMatrix != null && !angularSelectionMatrix.epsilonEquals(other.angularSelectionMatrix, epsilon))
      {
         return false;
      }

      if (linearWeightMatrix == null ^ other.linearWeightMatrix == null)
      {
         return false;
      }

      if (linearWeightMatrix != null && !linearWeightMatrix.epsilonEquals(other.linearWeightMatrix, epsilon))
      {
         return false;
      }

      if (angularWeightMatrix == null ^ other.angularWeightMatrix == null)
      {
         return false;
      }

      if (angularWeightMatrix != null && !angularWeightMatrix.epsilonEquals(other.angularWeightMatrix, epsilon))
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

   public void setQueueingProperties(QueueableMessage queueingProperties)
   {
      this.queueingProperties = queueingProperties;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }
}
