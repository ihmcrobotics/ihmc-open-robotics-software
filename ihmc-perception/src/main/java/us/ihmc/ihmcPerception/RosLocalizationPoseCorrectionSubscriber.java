package us.ihmc.ihmcPerception;

import java.util.function.LongUnaryOperator;

import org.ros.node.NodeConfiguration;

import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPoseStampedSubscriber;

public class RosLocalizationPoseCorrectionSubscriber
{
   private static final boolean DEBUG = false;

   NodeConfiguration nodeConfig = NodeConfiguration.newPrivate();

   public RosLocalizationPoseCorrectionSubscriber(final RosMainNode rosMainNode, final PacketCommunicator controllerCommunicationBridge,
                                                  LongUnaryOperator robotMonotonicTimeCalculator)
   {

      RosPoseStampedSubscriber rosPoseStampedSubscriber = new RosPoseStampedSubscriber()
      {
         @Override
         protected void newPose(String frameID, TimeStampedTransform3D timeStampedTransform)
         {
            long timestamp = robotMonotonicTimeCalculator.applyAsLong(timeStampedTransform.getTimeStamp());
            timeStampedTransform.setTimeStamp(timestamp);

            StampedPosePacket posePacket = HumanoidMessageTools.createStampedPosePacket(frameID,
                                                                                        timeStampedTransform,
                                                                                        RosLocalizationConstants.DEFAULT_OVERLAP);
            posePacket.setDestination(PacketDestination.CONTROLLER.ordinal());
            if (DEBUG)
               System.out.println("Pose update received. \ntimestamp: " + timeStampedTransform.getTimeStamp());

            controllerCommunicationBridge.send(posePacket);
         }
      };

      rosMainNode.attachSubscriber(RosLocalizationConstants.POSE_UPDATE_TOPIC, rosPoseStampedSubscriber);
   }
}
