package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.h264.OpenH264Decoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.video.VideoCallback;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class H264CompressedVideoDataClient implements CompressedVideoDataClient
{
   private final VideoCallback videoStreamer;
   private final OpenH264Decoder decoder;
   private final ByteBuffer nalBuffer = ByteBuffer.allocateDirect(1048576);   // Decoder cannot handle frames > 1mb anyway
   private final YUVPictureConverter converter = new YUVPictureConverter();
   
   private BufferedImage image;
   
   H264CompressedVideoDataClient(VideoCallback videoStreamer)
   {
      this.videoStreamer = videoStreamer;
      this.decoder = new OpenH264Decoder();
   }

   @Override
   public synchronized void onFrame(VideoSource videoSource, byte[] data, long timestamp, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      nalBuffer.clear();
      nalBuffer.put(data);
      nalBuffer.clear();
      
      YUVPicture frame = decoder.decodeFrame(nalBuffer);
      if(frame != null)
      {
         image = converter.toBufferedImage(frame, image);
         videoStreamer.onFrame(videoSource, image, timestamp, position, orientation, intrinsicParameters);
         frame.delete();
      }
   }

   @Override
   public synchronized void connected()
   {
   }

   @Override
   public void disconnected()
   {
   }
}
