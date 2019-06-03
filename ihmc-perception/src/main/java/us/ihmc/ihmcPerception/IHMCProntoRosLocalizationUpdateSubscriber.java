package us.ihmc.ihmcPerception;

import java.util.function.LongUnaryOperator;

import org.ros.node.NodeConfiguration;

import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosNavMsgsOdometrySubscriber;

public class IHMCProntoRosLocalizationUpdateSubscriber
{
   private static final boolean DEBUG = false;
   private double overlap = 1.0;

   NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();
   
   public IHMCProntoRosLocalizationUpdateSubscriber(final RosMainNode rosMainNode, final PacketCommunicator packetCommunicator,
         LongUnaryOperator robotMonotonicTimeCalculator)
   {
	   
      RosNavMsgsOdometrySubscriber rosOdometrySubscriber = new RosNavMsgsOdometrySubscriber()
	   {
		   @Override
		   protected void newPose(String frameID,
				   TimeStampedTransform3D timeStampedTransform)
		   {
			   long timestamp = robotMonotonicTimeCalculator.applyAsLong(timeStampedTransform.getTimeStamp());
			   timeStampedTransform.setTimeStamp(timestamp);
			   
			   StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket(frameID, timeStampedTransform, overlap);
			   posePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
			   if (DEBUG) System.out.println("Pose update received. \ntimestamp: " + timeStampedTransform.getTimeStamp());
			   
			   packetCommunicator.send(posePacket);
		   }
	   };
	   
	   rosMainNode.attachSubscriber(RosLocalizationConstants.NAV_ODOMETRY_UPDATE_TOPIC, rosOdometrySubscriber);
   }
}
