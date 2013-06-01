package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;

import us.ihmc.graphics3DAdapter.camera.VideoSettings.VideoCompressionKey;
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

public class CompressedVideoDataClient implements ObjectConsumer<VideoPacket>, NetStateListener
{
   private final VideoStreamer videoStreamer;
   private VideoCompressionKey videoCompressionKey = null;
   private IStreamCoder inputStreamCoder;
   private IVideoPicture pictureIn;
   private IConverter converter;

   public CompressedVideoDataClient(ObjectCommunicator objectCommunicator, VideoStreamer videoStreamer)
   {
      this.videoStreamer = videoStreamer;

      objectCommunicator.attachStateListener(this);
      objectCommunicator.attachListener(VideoPacket.class, this);


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

   public synchronized void consumeObject(VideoPacket packetData)
   {
      if(inputStreamCoder == null || videoCompressionKey != packetData.getVideoCompressionKey())
      {
         initialize(packetData.getVideoCompressionKey());
      }
      
      IPacket packet = IPacket.make(IBuffer.make(null, packetData.getData(), 0, packetData.getData().length));
      inputStreamCoder.decodeVideo(pictureIn, packet, 0);

      if (pictureIn.isComplete())
      {
         videoStreamer.updateImage(converter.toImage(pictureIn), packetData.getPosition(), packetData.getOrientation(), packetData.getFieldOfView());
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

}
