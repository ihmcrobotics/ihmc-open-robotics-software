package us.ihmc.communication.producers;

import java.io.IOException;
import java.nio.ByteBuffer;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.codecs.h264.OpenH264Decoder;
import us.ihmc.codecs.yuv.YUVPicture;
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

   public CompressedVideoDataClient(VideoStreamer videoStreamer)
   {
      this.videoStreamer = videoStreamer;
      try
      {
         this.decoder = new OpenH264Decoder();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public synchronized void consumeObject(byte[] data, Point3d position, Quat4d orientation, double fov)
   {
      nalBuffer.clear();
      nalBuffer.put(data);
      nalBuffer.clear();
      
      YUVPicture frame;
      try
      {
         frame = decoder.decodeFrame(nalBuffer);
         if(frame != null)
         {
            videoStreamer.updateImage(frame.getImage(), position, orientation, fov);
         }
      }
      catch (IOException e)
      {
         System.err.println(e.getMessage());
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
