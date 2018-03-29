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

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/so3_trajectory")
public final class SO3TrajectoryMessage extends Packet<SO3TrajectoryMessage>
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory. Use dataFrame to define what frame the points are expressed in")
   public RecyclingArrayListPubSub<SO3TrajectoryPointMessage> taskspaceTrajectoryPoints = new RecyclingArrayListPubSub<>(SO3TrajectoryPointMessage.class,
                                                                                                                         SO3TrajectoryPointMessage::new, 5);

   @RosExportedField(documentation = "Frame information for this message.")
   public FrameInformation frameInformation = new FrameInformation();

   @RosExportedField(documentation = "The selection matrix for each axis.")
   public SelectionMatrix3DMessage selectionMatrix = new SelectionMatrix3DMessage();

   @RosExportedField(documentation = "The weight matrix for each axis.")
   public WeightMatrix3DMessage weightMatrix = new WeightMatrix3DMessage();

   @RosExportedField(documentation = "Flag that tells the controller whether the use of a custom control frame is requested.")
   public boolean useCustomControlFrame = false;

   @RosExportedField(documentation = "Pose of custom control frame. This is the frame attached to the rigid body that the taskspace trajectory is defined for.")
   public Pose3D controlFramePose = new Pose3D();

   @RosExportedField(documentation = "Properties for queueing trajectories.")
   public QueueableMessage queueingProperties = new QueueableMessage();

   /**
    * Empty constructor for serialization.
    */
   public SO3TrajectoryMessage()
   {
      super();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public SO3TrajectoryMessage(SO3TrajectoryMessage other)
   {
      set(other);
   }

   @Override
   public void set(SO3TrajectoryMessage other)
   {
      MessageTools.copyData(other.taskspaceTrajectoryPoints, taskspaceTrajectoryPoints);
      frameInformation.set(other.getFrameInformation());
      selectionMatrix.set(other.selectionMatrix);
      weightMatrix.set(other.weightMatrix);
      useCustomControlFrame = other.useCustomControlFrame;
      controlFramePose.set(other.controlFramePose);
      queueingProperties.set(other.queueingProperties);
      setPacketInformation(other);
   }

   public final RecyclingArrayListPubSub<SO3TrajectoryPointMessage> getTaskspaceTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   public FrameInformation getFrameInformation()
   {
      return frameInformation;
   }

   @Override
   public String toString()
   {
      if (taskspaceTrajectoryPoints != null)
         return getClass().getSimpleName() + ": number of SO3 trajectory points = " + taskspaceTrajectoryPoints.size() + "\n" + frameInformation.toString();
      else
         return getClass().getSimpleName() + ": no SO3 trajectory points";
   }

   @Override
   public boolean epsilonEquals(SO3TrajectoryMessage other, double epsilon)
   {
      if (!queueingProperties.epsilonEquals(other.queueingProperties, epsilon))
         return false;

      if (!frameInformation.epsilonEquals(other.frameInformation, epsilon))
         return false;

      if (!MessageTools.epsilonEquals(taskspaceTrajectoryPoints, other.taskspaceTrajectoryPoints, epsilon))
         return false;

      if (selectionMatrix == null ^ other.selectionMatrix == null)
         return false;

      if (selectionMatrix == null && !selectionMatrix.epsilonEquals(other.selectionMatrix, epsilon))
         return false;

      if (weightMatrix == null ^ other.weightMatrix == null)
         return false;

      if (weightMatrix == null && !weightMatrix.epsilonEquals(other.weightMatrix, epsilon))
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
