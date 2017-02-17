package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.h264.OpenH264Decoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

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
   public synchronized void consumeObject(byte[] data, Point3D position, Quaternion orientation, IntrinsicParameters intrinsicParameters)
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
