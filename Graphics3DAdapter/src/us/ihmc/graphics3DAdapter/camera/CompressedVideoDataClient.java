package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;

import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectCommunicator;

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
   private final VideoSettings settings;

   private IStreamCoder inputStreamCoder;
   private final IVideoPicture pictureIn;
   private final IConverter converter;

   public CompressedVideoDataClient(VideoSettings settings, ObjectCommunicator objectCommunicator, VideoStreamer videoStreamer)
   {
      this.videoStreamer = videoStreamer;
      this.settings = settings;

      objectCommunicator.attachStateListener(this);
      objectCommunicator.attachListener(VideoPacket.class, this);

      if (objectCommunicator.isConnected())
      {
         throw new RuntimeException("Do not connect the ObjectCommunicator before the video server is live");
      }

      pictureIn = IVideoPicture.make(settings.getColorSpace(), settings.getWidth(), settings.getHeight());

      BufferedImage bufferedImage = new BufferedImage(settings.getWidth(), settings.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
      converter = ConverterFactory.createConverter(bufferedImage, settings.getColorSpace());
   }


   public synchronized void consumeObject(VideoPacket packetData)
   {
      if (inputStreamCoder != null)
      {
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

   public void disconnected()
   {
      close();
   }

}
