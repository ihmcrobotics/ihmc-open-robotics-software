package us.ihmc.avatar.stereoVision.net.udp;

import us.ihmc.avatar.stereoVision.net.packet.StereoVisionPacket;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;

public final class StereoVisionUDPPacketUtil
{
   public static final String IP_DESTINATION = System.getProperty("stereo.vision.udp.dest.address", "127.0.0.1");
   public static final String IP_BIND = System.getProperty("stereo.vision.udp.bind.address", "127.0.0.1");
   public static final int UDP_PORT = 9201;
   public static final int IPV4_HEADER_LENGTH = 28;
   public static final int DATAGRAM_MAX_LENGTH = (int) (Math.pow(2, 16) - 1) - IPV4_HEADER_LENGTH;
   public static final int STEREO_VISION_PACKET_HEADER_LENGTH = 1;
   public static final int STEREO_VISION_MAX_PACKET_DATA_LENGTH = DATAGRAM_MAX_LENGTH - STEREO_VISION_PACKET_HEADER_LENGTH;

   public static void send(StereoVisionPacket packet, DatagramSocket datagramSocket, InetAddress sendAddress)
   {
      byte[] datagramData = new byte[DATAGRAM_MAX_LENGTH];
      ByteBuffer datagramBuffer = ByteBuffer.wrap(datagramData);

      // Write header data (no longer than STEREO_VISION_PACKET_HEADER_LENGTH)
      datagramBuffer.put(packet.getPacketID());

      // Write packet specific data
      int packetDataLength = packet.serialize(datagramBuffer);

      if (packetDataLength > STEREO_VISION_MAX_PACKET_DATA_LENGTH)
      {
         throw new RuntimeException("packetDataLength too large");
      }

      DatagramPacket datagramPacket = new DatagramPacket(datagramData, packetDataLength + STEREO_VISION_PACKET_HEADER_LENGTH, sendAddress, UDP_PORT);
      try
      {
         datagramSocket.send(datagramPacket);
      }
      catch (IOException e)
      {
         LogTools.error(e);
      }
   }
}
