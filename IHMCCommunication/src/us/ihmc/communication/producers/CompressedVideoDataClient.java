package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.h264.OpenH264Decoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.utilities.VideoStreamer;

public class CompressedVideoDataClient implements NetStateListener
{
   private final VideoStreamer videoStreamer;
   private final OpenH264Decoder decoder;
   private final ByteBuffer nalBuffer = ByteBuffer.allocateDirect(1048576);   // Decoder cannot handle frames > 1mb anyway
   private final YUVPictureConverter converter = new YUVPictureConverter();
   
   private BufferedImage image;
   
   public CompressedVideoDataClient(VideoStreamer videoStreamer)
   {
      this.videoStreamer = videoStreamer;
      this.decoder = new OpenH264Decoder();
   }

   public synchronized void consumeObject(byte[] data, Point3d position, Quat4d orientation, double fov)
   {
      nalBuffer.clear();
      nalBuffer.put(data);
      nalBuffer.clear();
      
      YUVPicture frame = decoder.decodeFrame(nalBuffer);
      if(frame != null)
      {
         image = converter.toBufferedImage(frame, image);
         videoStreamer.updateImage(image, position, orientation, fov);
         frame.delete();
      }
      
      
   }

   public synchronized void close()
   {
   }

   public synchronized void connected()
   {
   }

   public void disconnected()
   {
      close();
   }
   
   public void attachVideoPacketListener(PacketCommunicator communicator)
   {
      communicator.attachListener(VideoPacket.class, new PacketConsumer<VideoPacket>()
      {
         public void receivedPacket(VideoPacket object)
         {
            CompressedVideoDataClient.this.consumeObject(object.getData(), object.getPosition(), object.getOrientation(), object.getFieldOfView());
         }
      });
      communicator.attachStateListener(this);
   }

}
