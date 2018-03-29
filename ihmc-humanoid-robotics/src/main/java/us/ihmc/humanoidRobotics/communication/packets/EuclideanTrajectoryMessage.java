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

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/euclidean_trajectory")
public final class EuclideanTrajectoryMessage extends Packet<EuclideanTrajectoryMessage>
{
   @RosExportedField(documentation = "List of trajectory points (in taskpsace) to go through while executing the trajectory.")
   public RecyclingArrayListPubSub<EuclideanTrajectoryPointMessage> taskspaceTrajectoryPoints = new RecyclingArrayListPubSub<>(EuclideanTrajectoryPointMessage.class,
                                                                                                                               EuclideanTrajectoryPointMessage::new,
                                                                                                                               5);

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
      set(other);
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
    * Returns the internal mutable list of points, modifying this list changes the internal message
    * 
    * @return
    */
   public final RecyclingArrayListPubSub<EuclideanTrajectoryPointMessage> getTaskspaceTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   public FrameInformation getFrameInformation()
   {
      return frameInformation;
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

   public boolean getUseCustomControlFrame()
   {
      return useCustomControlFrame;
   }

   public QueueableMessage getQueueingProperties()
   {
      return queueingProperties;
   }
}