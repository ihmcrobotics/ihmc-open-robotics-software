package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.h264.OpenH264Decoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import boofcv.struct.calib.IntrinsicParameters;

public class H264CompressedVideoDataClient implements CompressedVideoDataClient
{
   private final VideoStreamer videoStreamer;
   private final OpenH264Decoder decoder;
   private final ByteBuffer nalBuffer = ByteBuffer.allocateDirect(1048576);   // Decoder cannot handle frames > 1mb anyway
   private final YUVPictureConverter converter = new YUVPictureConverter();
   
   private BufferedImage image;
   
   H264CompressedVideoDataClient(VideoStreamer videoStreamer)
   {
      this.videoStreamer = videoStreamer;
      this.decoder = new OpenH264Decoder();
   }

   @Override
   public synchronized void consumeObject(byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters)
   {
      nalBuffer.clear();
      nalBuffer.put(data);
      nalBuffer.clear();
      
      YUVPicture frame = decoder.decodeFrame(nalBuffer);
      if(frame != null)
      {
         image = converter.toBufferedImage(frame, image);
         videoStreamer.updateImage(image, position, orientation, intrinsicParameters);
         frame.delete();
      }
      
      
   }

   public synchronized void connected()
   {
   }

   public void disconnected()
   {
   }
   
}
