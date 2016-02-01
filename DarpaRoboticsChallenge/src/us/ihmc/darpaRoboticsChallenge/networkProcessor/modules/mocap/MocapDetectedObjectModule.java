package us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.mocap;

import java.util.ArrayList;

import optiTrack.MocapDataClient;
import optiTrack.MocapRigidBody;
import optiTrack.MocapRigidbodiesListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;


/**
 * This module converts MocapRigidBodies to robot world and sends them as packets the UI understands
 */
public class MocapDetectedObjectModule implements MocapRigidbodiesListener
{
   private final PacketCommunicator mocapModulePacketCommunicator;
   private final MocapToStateEstimatorFrameConverter frameConverter;
   
   public MocapDetectedObjectModule(MocapDataClient mocapDataClient, MocapToStateEstimatorFrameConverter frameConverter, PacketCommunicator packetCommunicator)
   {
      mocapModulePacketCommunicator = packetCommunicator;
      this.frameConverter = frameConverter;
      mocapDataClient.registerRigidBodiesListener(this);
   }
   
   @Override
   public void updateRigidbodies(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for(int i = 0; i < listOfRigidbodies.size(); i++)
      {
         MocapRigidBody mocapObject = listOfRigidbodies.get(i);
         int id = mocapObject.getId();
         
         RigidBodyTransform pose = new RigidBodyTransform();
         mocapObject.getPose(pose);
         
         pose = frameConverter.convertMocapPoseToRobotFrame(mocapObject);
         
         DetectedObjectPacket detectedMocapObject = new DetectedObjectPacket(pose, id);
         mocapModulePacketCommunicator.send(detectedMocapObject);
      }
   }
}
