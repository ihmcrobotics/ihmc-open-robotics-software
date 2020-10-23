package us.ihmc.atlas.behaviors;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.function.Consumer;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class SCSVideoDataROS2Bridge implements VideoDataServer
{
   private static final Object hackyLockBecauseJPEGEncoderIsNotThreadsafe = new Object();

   private final YUVPictureConverter converter = new YUVPictureConverter();
   private final JPEGEncoder encoder = new JPEGEncoder();
   private final Consumer<VideoPacket> scsCameraPublisher;

   public SCSVideoDataROS2Bridge(Consumer<VideoPacket> scsCameraPublisher)
   {
      this.scsCameraPublisher = scsCameraPublisher;
   }

   @Override
   public void onFrame(VideoSource videoSource,
                       BufferedImage bufferedImage,
                       long timeStamp,
                       Point3DReadOnly cameraPosition,
                       QuaternionReadOnly cameraOrientation,
                       CameraPinholeBrown intrinsicParameters)
   {

      YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVPicture.YUVSubsamplingType.YUV420);
      try
      {
         ByteBuffer buffer;
         synchronized (hackyLockBecauseJPEGEncoderIsNotThreadsafe)
         {
            buffer = encoder.encode(picture, 75);
         }
         byte[] data = new byte[buffer.remaining()];
         buffer.get(data);
         VideoPacket videoPacket = HumanoidMessageTools.createVideoPacket(videoSource,
                                                                          timeStamp,
                                                                          data,
                                                                          cameraPosition,
                                                                          cameraOrientation,
                                                                          intrinsicParameters);
         scsCameraPublisher.accept(videoPacket);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      picture.delete();
   }

   @Override
   public boolean isConnected()
   {
      return true; // do nothing
   }
}
