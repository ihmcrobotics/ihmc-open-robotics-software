package us.ihmc.ihmcPerception;

import org.ros.node.NodeConfiguration;

import std_msgs.Float64;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.sensing.LocalizationStatusPacket;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPoseStampedSubscriber;

public class IHMCETHRosLocalizationUpdateSubscriber
{
   private static final boolean DEBUG = false;
   private double overlap = 1.0;

   NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
   
   public IHMCETHRosLocalizationUpdateSubscriber(final RosMainNode rosMainNode, final PacketCommunicator rosModulePacketCommunicator,
         final PPSTimestampOffsetProvider ppsTimeOffsetProvider)
   {
	   
	   RosPoseStampedSubscriber rosPoseStampedSubscriber = new RosPoseStampedSubscriber()
	   {
		   @Override
		   protected void newPose(String frameID,
				   TimeStampedTransform3D timeStampedTransform)
		   {
			   long timestamp = timeStampedTransform.getTimeStamp();
			   timestamp = ppsTimeOffsetProvider.adjustTimeStampToRobotClock(timestamp);
			   timeStampedTransform.setTimeStamp(timestamp);
			   
			   StampedPosePacket posePacket = new StampedPosePacket(frameID, timeStampedTransform, overlap);
			   posePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
			   if (DEBUG) System.out.println("Pose update received. \ntimestamp: " + timeStampedTransform.getTimeStamp());
			   
			   rosModulePacketCommunicator.send(posePacket);
		   }
	   };
	   
	   rosMainNode.attachSubscriber(RosLocalizationConstants.POSE_UPDATE_TOPIC, rosPoseStampedSubscriber);

      AbstractRosTopicSubscriber<Float64> overlapSubscriber = new AbstractRosTopicSubscriber<std_msgs.Float64>(std_msgs.Float64._TYPE) {
         @Override
         public void onNewMessage(std_msgs.Float64 message) {
            overlap = message.getData();
            LocalizationStatusPacket localizationOverlapPacket = new LocalizationStatusPacket(overlap,null);
            rosModulePacketCommunicator.send(localizationOverlapPacket);
         }
      };
      rosMainNode.attachSubscriber(RosLocalizationConstants.OVERLAP_UPDATE_TOPIC, overlapSubscriber);

      AbstractRosTopicSubscriber<std_msgs.String> statusSubscriber = new AbstractRosTopicSubscriber<std_msgs.String>(std_msgs.String._TYPE) {

         @Override
         public void onNewMessage(std_msgs.String message) {
            String status = message.getData();
            LocalizationStatusPacket localizationOverlapPacket = new LocalizationStatusPacket(overlap,status);
            rosModulePacketCommunicator.send(localizationOverlapPacket);
         }

      };

      rosMainNode.attachSubscriber(RosLocalizationConstants.STATUS_UPDATE_TOPIC, statusSubscriber);
   }
}
