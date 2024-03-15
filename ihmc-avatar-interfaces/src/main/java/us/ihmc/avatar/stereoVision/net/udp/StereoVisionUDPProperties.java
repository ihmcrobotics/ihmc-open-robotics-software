package us.ihmc.avatar.stereoVision.net.udp;

public class StereoVisionUDPProperties
{
   public static final String IP_DESTINATION = System.getProperty("stereo.vision.udp.dest.address", "127.0.0.1");
   public static final String IP_BIND = System.getProperty("stereo.vision.udp.bind.address", "127.0.0.1");
   public static final int UDP_PORT = 9201;
   public static final int IPV4_HEADER_LENGTH = 28;
   public static final int DATAGRAM_MAX_LENGTH = (int) (Math.pow(2, 16) - 1) - IPV4_HEADER_LENGTH;
   public static final int DATAGRAM_MAX_IMAGE_DATA_LENGTH = DATAGRAM_MAX_LENGTH - 256;
}
