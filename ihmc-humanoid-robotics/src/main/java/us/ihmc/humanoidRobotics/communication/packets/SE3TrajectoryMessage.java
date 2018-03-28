package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.QueueableMessage;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.communication.packets.WeightMatrix3DMessage;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.idl.RecyclingArrayListPubSub;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/se3_trajectory")
public final class SE3TrajectoryMessage extends Packet<SE3TrajectoryMessage>
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public RecyclingArrayListPubSub<SE3TrajectoryPointMessage> taskspaceTrajectoryPoints = new RecyclingArrayListPubSub<>(SE3TrajectoryPointMessage.class,
                                                                                                                 SE3TrajectoryPointMessage::new, 5);
   @RosExportedField(documentation = "The selection matrix for each axis of the angular part.")
   public SelectionMatrix3DMessage angularSelectionMatrix = new SelectionMatrix3DMessage();
   @RosExportedField(documentation = "The selection matrix for each axis of the linear part.")
   public SelectionMatrix3DMessage linearSelectionMatrix = new SelectionMatrix3DMessage();

   @RosExportedField(documentation = "Frame information for this message.")
   public FrameInformation frameInformation = new FrameInformation();

   @RosExportedField(documentation = "The weight matrix for each axis of the angular part.")
   public WeightMatrix3DMessage angularWeightMatrix = new WeightMatrix3DMessage();
   @RosExportedField(documentation = "The weight matrix for each axis of the linear part.")
   public WeightMatrix3DMessage linearWeightMatrix = new WeightMatrix3DMessage();

   @RosExportedField(documentation = "Flag that tells the controller whether the use of a custom control frame is requested.")
   public boolean useCustomControlFrame = false;

   @RosExportedField(documentation = "Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.")
   public Pose3D controlFramePose = new Pose3D();
   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   public SE3TrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public SE3TrajectoryMessage(SE3TrajectoryMessage other)
   {
      set(other);
   }

   @Override
   public void set(SE3TrajectoryMessage other)
   {
      MessageTools.copyData(other.taskspaceTrajectoryPoints, taskspaceTrajectoryPoints);
      frameInformation.set(other.getFrameInformation());
      angularSelectionMatrix.set(other.angularSelectionMatrix);
      linearSelectionMatrix.set(other.linearSelectionMatrix);
      angularWeightMatrix.set(other.angularWeightMatrix);
      linearWeightMatrix.set(other.linearWeightMatrix);
      useCustomControlFrame = other.useCustomControlFrame;
      controlFramePose.set(other.controlFramePose);
      queueingProperties.set(other.queueingProperties);
      setPacketInformation(other);
   }

   public final RecyclingArrayListPubSub<SE3TrajectoryPointMessage> getTaskspaceTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   public FrameInformation getFrameInformation()
   {
      return frameInformation;
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

   public boolean getUseCustomControlFrame()
   {
      return useCustomControlFrame;
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
