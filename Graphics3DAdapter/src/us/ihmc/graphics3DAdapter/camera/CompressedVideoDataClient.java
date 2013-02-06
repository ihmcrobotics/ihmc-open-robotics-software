package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.net.Socket;
import java.net.UnknownHostException;

import us.ihmc.utilities.ThreadTools;

import com.xuggle.ferry.IBuffer;
import com.xuggle.xuggler.IPacket;
import com.xuggle.xuggler.IStreamCoder;
import com.xuggle.xuggler.IStreamCoder.Direction;
import com.xuggle.xuggler.IVideoPicture;
import com.xuggle.xuggler.video.ConverterFactory;
import com.xuggle.xuggler.video.IConverter;

public class CompressedVideoDataClient implements Runnable
{
   private final String host;
   private final int port;
   private final VideoSettings settings;
   private final VideoStreamer videoStreamer;
   
   public CompressedVideoDataClient(VideoSettings settings, String host, int port, VideoStreamer videoStreamer)
   {
      this.settings = settings;
      this.host = host;
      this.port = port;
      this.videoStreamer = videoStreamer;
   }
   
   public void run()
   {
      
      
      Socket clientSocket;
      ObjectInputStream input;
      do
      {
         try
         {
            clientSocket = new Socket(host, port);
            input = new ObjectInputStream(clientSocket.getInputStream());
            break;
         }
         catch (UnknownHostException e)
         {
            throw new RuntimeException("Cannot find host " + host);
         }
         catch (IOException e)
         {
            ThreadTools.sleep(1000);
            continue;
         }
      }while(true);
      System.out.println("Connected videoDataClient");
      

      IStreamCoder inputStreamCoder = IStreamCoder.make(Direction.DECODING, settings.getCodec());
      inputStreamCoder.setNumPicturesInGroupOfPictures(settings.getNumberOfPicturesInGroupOfPictures());
      inputStreamCoder.setWidth(settings.getWidth());
      inputStreamCoder.setHeight(settings.getHeight());
      
      inputStreamCoder.setPixelType(settings.getColorSpace());
                  
      inputStreamCoder.setTimeBase(settings.getTimeBase());

      inputStreamCoder.open(null, null);
      
      
      
      
      IVideoPicture pictureIn = IVideoPicture.make(settings.getColorSpace(), settings.getWidth(), settings.getHeight());
      
      BufferedImage bufferedImage = new BufferedImage(settings.getWidth(), settings.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
      IConverter converter = ConverterFactory.createConverter(bufferedImage, settings.getColorSpace());

      
      while(true)
      {
         try
         {
            VideoPacket packetData = (VideoPacket) input.readObject();
            IPacket packet = IPacket.make(IBuffer.make(null, packetData.getData(), 0, packetData.getData().length));
            inputStreamCoder.decodeVideo(pictureIn, packet, 0);
            
            if(pictureIn.isComplete())
            {
               videoStreamer.updateImage(converter.toImage(pictureIn), packetData.getPosition(), packetData.getOrientation(), packetData.getFieldOfView());
            }
            else
            {
               System.out.println("Video packet is not complete");
            }
         }
         catch(IOException e)
         {
            System.err.println("Connection lost");
            break;
         }
         catch (ClassNotFoundException e)
         {
            System.err.println(e.getMessage());
         }
      }
   }
   
}
