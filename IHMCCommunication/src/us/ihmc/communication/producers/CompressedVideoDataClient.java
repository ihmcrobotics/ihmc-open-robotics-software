package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.sensing.VideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoCompressionKey;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.graphics3DAdapter.camera.VideoSettingsFactory;
import us.ihmc.graphics3DAdapter.camera.VideoStreamer;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

import com.xuggle.ferry.IBuffer;
import com.xuggle.xuggler.IPacket;
import com.xuggle.xuggler.IStreamCoder;
import com.xuggle.xuggler.IStreamCoder.Direction;
import com.xuggle.xuggler.IVideoPicture;
import com.xuggle.xuggler.video.ConverterFactory;
import com.xuggle.xuggler.video.IConverter;

public class CompressedVideoDataClient implements NetStateListener
{
   private final VideoStreamer videoStreamer;
   private VideoCompressionKey videoCompressionKey = null;
   private IStreamCoder inputStreamCoder;
   private IVideoPicture pictureIn;
   private IConverter converter;

   public CompressedVideoDataClient(VideoStreamer videoStreamer)
   {
      this.videoStreamer = videoStreamer;
   }

   
   public void initialize(VideoSettings settings)
   {
      pictureIn = IVideoPicture.make(settings.getColorSpace(), settings.getWidth(), settings.getHeight());
      
      BufferedImage bufferedImage = new BufferedImage(settings.getWidth(), settings.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
      converter = ConverterFactory.createConverter(bufferedImage, settings.getColorSpace());
      
      if (inputStreamCoder != null)
      {
         inputStreamCoder.close();
      }
      
      inputStreamCoder = IStreamCoder.make(Direction.DECODING, settings.getCodec());
      inputStreamCoder.setNumPicturesInGroupOfPictures(settings.getNumberOfPicturesInGroupOfPictures());
      inputStreamCoder.setWidth(settings.getWidth());
      inputStreamCoder.setHeight(settings.getHeight());
      
      inputStreamCoder.setPixelType(settings.getColorSpace());
      
      inputStreamCoder.setTimeBase(settings.getTimeBase());
      
      inputStreamCoder.open(null, null);
      
   }
   
   private void initialize(VideoCompressionKey videoCompressionKey)
   {
      System.out.println("Initializing receiving video to " + videoCompressionKey);
      initialize(VideoSettingsFactory.getSettings(videoCompressionKey));
      this.videoCompressionKey = videoCompressionKey; 
   }

   public synchronized void consumeObject(VideoCompressionKey videoCompressionKey, byte[] data, Point3d position, Quat4d orientation, double fov)
   {
      if(inputStreamCoder == null || this.videoCompressionKey != videoCompressionKey)
      {
         initialize(videoCompressionKey);
      }
      
      IPacket packet = IPacket.make(IBuffer.make(null, data, 0, data.length));
      inputStreamCoder.decodeVideo(pictureIn, packet, 0);

      if (pictureIn.isComplete())
      {
         videoStreamer.updateImage(converter.toImage(pictureIn), position, orientation, fov);
      }
      else
      {
         System.out.println("Video packet is not complete");
      }
   }

   public synchronized void close()
   {
      if (inputStreamCoder != null)
      {
         inputStreamCoder.close();
         inputStreamCoder = null;
      }
   }

   public synchronized void connected()
   {
   }

   public void disconnected()
   {
      close();
   }
   
   public void attachVideoPacketListener(ObjectCommunicator communicator)
   {
      communicator.attachListener(VideoPacket.class, new ObjectConsumer<VideoPacket>()
      {
         public void consumeObject(VideoPacket object)
         {
            CompressedVideoDataClient.this.consumeObject(object.getVideoCompressionKey(), object.getData(), object.getPosition(), object.getOrientation(), object.getFieldOfView());
         }
      });
      communicator.attachStateListener(this);
   }

}
