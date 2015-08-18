package us.ihmc.communication.producers;

public class CompressedVideoDataFactory
{
   public enum Algorithm
   {
      JPEG, H264
   }

   public static final Algorithm algorithm = Algorithm.JPEG;

   public static CompressedVideoDataServer createCompressedVideoDataServer(CompressedVideoHandler handler)
   {
      switch (algorithm)
      {
      case H264:
         H264CompressedVideoDataServer h264Server = new H264CompressedVideoDataServer(handler);
         return h264Server;
      case JPEG:
         return new JPEGCompressedVideoDataServer(handler);
      default:
         throw new RuntimeException("Unknown algorithm");

      }
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
