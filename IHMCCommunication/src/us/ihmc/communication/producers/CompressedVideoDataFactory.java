package us.ihmc.communication.producers;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.VideoControlPacket;
import us.ihmc.utilities.VideoStreamer;

public class CompressedVideoDataFactory
{
   public enum Algorithm
   {
      JPEG, H264
   }

   public static final Algorithm algorithm = Algorithm.JPEG;

   public static CompressedVideoDataServer createCompressedVideoDataServer(PacketCommunicator sensorSuitePacketCommunicator,
         CompressedVideoHandler handler)
   {
      switch (algorithm)
      {
      case H264:
         H264CompressedVideoDataServer h264Server = new H264CompressedVideoDataServer(handler);
         if (sensorSuitePacketCommunicator != null)
         {
            sensorSuitePacketCommunicator.attachListener(VideoControlPacket.class, h264Server);
         }
         return h264Server;
      case JPEG:
         return new JPEGCompressedVideoDataServer(handler);
      default:
         throw new RuntimeException("Unknown algorithm");

      }
   }

   public static CompressedVideoDataServer createCompressedVideoDataServer(CompressedVideoHandler handler)
   {
      return createCompressedVideoDataServer(null, handler);
   }

   public static CompressedVideoDataClient createCompressedVideoDataClient(VideoStreamer videoStreamer)
   {
      switch (algorithm)
      {
      case H264:
         return new H264CompressedVideoDataClient(videoStreamer);
      case JPEG:
         return new JPEGCompressedVideoDataClient(videoStreamer);
      default:
         throw new RuntimeException("Unknown algorithm");
      }
   }

}
