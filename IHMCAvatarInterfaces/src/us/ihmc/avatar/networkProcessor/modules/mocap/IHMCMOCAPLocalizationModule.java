package us.ihmc.avatar.networkProcessor.modules.mocap;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;

import java.util.ArrayList;

public class IHMCMOCAPLocalizationModule implements MocapRigidbodiesListener
{
   private final MocapToPelvisFrameConverter mocapToPelvisFrameConverter;
   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.MOCAP_MODULE, new IHMCCommunicationKryoNetClassList());

   public IHMCMOCAPLocalizationModule(DRCRobotModel drcRobotModel)
   {
      MocapDataClient mocapDataClient = new MocapDataClient();
      mocapDataClient.registerRigidBodiesListener(this);

      mocapToPelvisFrameConverter = new MocapToPelvisFrameConverter(drcRobotModel, packetCommunicator);
   }

   public PacketCommunicator getPacketCommunicator()
   {
      return packetCommunicator;
   }

   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody mocapRigidBody = listOfRigidbodies.get(i);
         RigidBodyTransform pelvisTransform = mocapToPelvisFrameConverter.convertMocapPoseToRobotFrame(mocapRigidBody);
         sendPelvisTransformToController(mocapRigidBody.getId(), pelvisTransform);
      }
   }

   private void sendPelvisTransformToController(int frameID, RigidBodyTransform pelvisTransform)
   {
      TimeStampedTransform3D timestampedTransform = createTimestampedTransform(pelvisTransform);
      StampedPosePacket stampedPosePacket = new StampedPosePacket(Integer.toString(frameID), timestampedTransform, 1.0);
      stampedPosePacket.setDestination(PacketDestination.CONTROLLER.ordinal());

      packetCommunicator.send(stampedPosePacket);
   }

   private TimeStampedTransform3D createTimestampedTransform(RigidBodyTransform rigidBodyTransform)
   {
      return new TimeStampedTransform3D(rigidBodyTransform, System.currentTimeMillis());
   }
}
