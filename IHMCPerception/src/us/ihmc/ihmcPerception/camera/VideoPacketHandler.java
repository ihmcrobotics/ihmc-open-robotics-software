package us.ihmc.ihmcPerception.camera;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.tools.io.printing.PrintTools;

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
   public void newVideoPacketAvailable(VideoSource videoSource, long timeStamp, byte[] data, Point3D position, Quaternion orientation, IntrinsicParameters intrinsicParameters)
   {
      if (DEBUG)
      {
         PrintTools.debug(DEBUG, this, "Sending new VideoPacket FPS: " + 1.0 / timer.averageLap());
         timer.lap();
      }
         
      packetCommunicator.send(new VideoPacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters, packetDestination));
   }

   @Override
   public void addNetStateListener(NetStateListener compressedVideoDataServer)
   {
      packetCommunicator.attachStateListener(compressedVideoDataServer);
   }

   @Override
   public boolean isConnected()
   {
      return packetCommunicator.isConnected();
   }
}