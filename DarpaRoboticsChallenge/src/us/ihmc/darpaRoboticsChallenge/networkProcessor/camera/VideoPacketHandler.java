package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.robotics.robotSide.RobotSide;

class VideoPacketHandler implements CompressedVideoHandler
{
   private final PacketCommunicator packetCommunicator;

   public VideoPacketHandler(PacketCommunicator sensorSuitePacketCommunicator)
   {
      this.packetCommunicator = sensorSuitePacketCommunicator;
   }

   public void newVideoPacketAvailable(RobotSide robotSide, long timeStamp, byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters)
   {
      if (CameraDataReceiver.DEBUG)
      {
         System.out.println(getClass().getName() + " sending new VideoPacket");
      }
      packetCommunicator.send(new VideoPacket(robotSide, timeStamp, data, position, orientation, intrinsicParameters));
   }

   public void addNetStateListener(NetStateListener compressedVideoDataServer)
   {
      packetCommunicator.attachStateListener(compressedVideoDataServer);
   }

   public boolean isConnected()
   {
      return packetCommunicator.isConnected();
   }

}