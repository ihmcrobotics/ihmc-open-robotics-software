package us.ihmc.ihmcPerception.camera;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;

public class VideoPacketHandler implements CompressedVideoHandler
{
   private static final boolean DEBUG = false;
   private final PacketCommunicator packetCommunicator;
   private final PacketDestination packetDestination;

   public VideoPacketHandler(PacketCommunicator sensorSuitePacketCommunicator)
   {
      this(sensorSuitePacketCommunicator, PacketDestination.BROADCAST);
   }

   public VideoPacketHandler(PacketCommunicator sensorSuitePacketCommunicator, PacketDestination packetDestination)
   {
      this.packetCommunicator = sensorSuitePacketCommunicator;
      this.packetDestination = packetDestination;
   }

   private Stopwatch timer;
   {
      if (DEBUG)
         timer = new Stopwatch().start();
   }
   
   @Override
   public void onFrame(VideoSource videoSource, byte[] data, long timeStamp, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      if (DEBUG)
      {
         PrintTools.debug(DEBUG, this, "Sending new VideoPacket FPS: " + 1.0 / timer.averageLap());
         timer.lap();
      }
         
      packetCommunicator.send(HumanoidMessageTools.createVideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters, packetDestination));
   }

   @Override
   public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
   {
      packetCommunicator.attachStateListener(compressedVideoDataServer);
   }

   @Override
   public boolean isConnected()
   {
      return packetCommunicator.isConnected();
   }
}