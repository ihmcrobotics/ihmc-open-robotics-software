package us.ihmc.perception.detections.foundationPose;

import perception_msgs.FoundationPoseParameters;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;

public class SyncedIsaacROSFoundationPoseParameters extends LatestTimestampModifiable
{
   private final FoundationPoseParameters message;

   private final ROS2Node ros2Node;
   private final ROS2Subscription<FoundationPoseParameters> subscription;
   private final ROS2Publisher<FoundationPoseParameters> publisher;

   private final CRDTBidirectionalBoolean enabled;          // Whether pose estimation is enabled for this object
   private final CRDTBidirectionalBoolean autoResetEnabled; // Whether auto reset is enabled
   private final CRDTBidirectionalDouble resetDistance;     // Distance threshold for resetting pose estimation

   public SyncedIsaacROSFoundationPoseParameters(ROS2Node ros2Node, CRDTInfo crdtInfo, IsaacROSFoundationPoseObject object)
   {
      super(crdtInfo);
      setModifierName("FoundationPose " + object.name() + " Parameters");

      this.ros2Node = ros2Node;
      message = new FoundationPoseParameters();

      subscription = ros2Node.createSubscription(object.topics.ihmcParameters(), reader -> this.fromMessage(reader.read()));
      publisher = ros2Node.createPublisher(object.topics.ihmcParameters());

      enabled = new CRDTBidirectionalBoolean(this, true);
      autoResetEnabled = new CRDTBidirectionalBoolean(this, true);
      resetDistance = new CRDTBidirectionalDouble(this, 0.1);

      // Request full data to initialize
      requestSendFullData();
   }

   public void update()
   {
      checkModified();

      if (pollNeedSendFullData())
      {
         toMessage(message);
         publisher.publish(message);
      }
   }

   public CRDTBidirectionalBoolean getEnabled()
   {
      return enabled;
   }

   public CRDTBidirectionalBoolean getAutoResetEnabled()
   {
      return autoResetEnabled;
   }

   public CRDTBidirectionalDouble getResetDistance()
   {
      return resetDistance;
   }

   public void close()
   {
      ros2Node.destroyPublisher(publisher);
      ros2Node.destroySubscription(subscription);
   }

   private void toMessage(FoundationPoseParameters messageToPack)
   {
      toMessage(messageToPack.getLatestTimestampModifiable());

      messageToPack.setEnabled(enabled.toMessage());
      messageToPack.setAutoResetEnabled(autoResetEnabled.toMessage());
      messageToPack.setResetDistance(resetDistance.toMessage());
   }

   private void fromMessage(FoundationPoseParameters messageToRead)
   {
      fromMessage(messageToRead.getLatestTimestampModifiable());

      enabled.fromMessage(messageToRead.getEnabled());
      autoResetEnabled.fromMessage(messageToRead.getAutoResetEnabled());
      resetDistance.fromMessage(messageToRead.getResetDistance());

      confirmReceivedFullData();
   }
}
