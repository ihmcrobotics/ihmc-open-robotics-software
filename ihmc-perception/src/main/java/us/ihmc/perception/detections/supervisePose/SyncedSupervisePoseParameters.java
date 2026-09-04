package us.ihmc.perception.detections.supervisePose;

import perception_msgs.FoundationPoseParameters;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalDouble;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

public class SyncedSupervisePoseParameters extends LatestTimestampModifiable
{
   private final ROS2Node ros2Node;
   private final FoundationPoseParameters message;

   private final ROS2Subscription<FoundationPoseParameters> subscription;
   private final ROS2Publisher<FoundationPoseParameters> publisher;

   private final CRDTBidirectionalBoolean enabled;
   private final CRDTBidirectionalBoolean autoResetEnabled;
   private final CRDTBidirectionalDouble resetDistance;

   public SyncedSupervisePoseParameters(ROS2Node ros2Node,
                                        CRDTInfo crdtInfo,
                                        SupervisePoseObject object)
   {
      this(ros2Node,
           crdtInfo,
           "FoundationPose " + object.category + "/" + object.instance + " Parameters",
           object.topics.ihmcParameters(),
           SupervisePoseResetDistanceLibrary.getResetDistance(object));
   }

   public SyncedSupervisePoseParameters(ROS2Node ros2Node,
                                        CRDTInfo crdtInfo,
                                        String modifierName,
                                        ROS2Topic<FoundationPoseParameters> parametersTopic,
                                        double defaultResetDistance)
   {
      super(crdtInfo);
      this.ros2Node = ros2Node;
      setModifierName(modifierName);

      message = new FoundationPoseParameters();

      subscription = ros2Node.createSubscription(parametersTopic, reader ->
      {
         FoundationPoseParameters receivedMessage = reader.read();
         if (receivedMessage != null)
            fromMessage(receivedMessage);
      });
      publisher = ros2Node.createPublisher(parametersTopic);

      enabled = new CRDTBidirectionalBoolean(this, true);
      autoResetEnabled = new CRDTBidirectionalBoolean(this, true);
      resetDistance = new CRDTBidirectionalDouble(this, defaultResetDistance);

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