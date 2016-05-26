package us.ihmc.ihmcPerception.camera;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.time.Timer;

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

   private Timer timer;
   {
      if (DEBUG)
         timer = new Timer().start();
   }
   
   @Override
   public void newVideoPacketAvailable(VideoSource videoSource, long timeStamp, byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters)
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