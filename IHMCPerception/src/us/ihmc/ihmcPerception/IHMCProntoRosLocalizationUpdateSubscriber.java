package us.ihmc.ihmcPerception;

import org.ros.node.NodeConfiguration;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosNavMsgsOdometrySubscriber;

public class IHMCProntoRosLocalizationUpdateSubscriber
{
   private static final boolean DEBUG = false;
   private double overlap = 1.0;

   NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
   
   public IHMCProntoRosLocalizationUpdateSubscriber(final RosMainNode rosMainNode, final PacketCommunicator packetCommunicator,
         final PPSTimestampOffsetProvider ppsTimeOffsetProvider)
   {
	   
      RosNavMsgsOdometrySubscriber rosOdometrySubscriber = new RosNavMsgsOdometrySubscriber()
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
			   
			   packetCommunicator.send(posePacket);
		   }
	   };
	   
	   rosMainNode.attachSubscriber(RosLocalizationConstants.NAV_ODOMETRY_UPDATE_TOPIC, rosOdometrySubscriber);
   }
}
