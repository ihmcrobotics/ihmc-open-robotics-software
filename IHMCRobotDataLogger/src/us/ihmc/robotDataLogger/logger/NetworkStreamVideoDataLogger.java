package us.ihmc.robotDataLogger.logger;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;

import javax.imageio.ImageIO;

import com.esotericsoftware.kryo.io.ByteBufferInputStream;

import us.ihmc.codecs.builder.MP4MJPEGMovieBuilder;
import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.robotDataLogger.LogDataHeader;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.gui.GUICaptureReceiver;

public class NetworkStreamVideoDataLogger extends VideoDataLoggerInterface implements LogPacketHandler
{
   private final static String description = "NetworkStream";
   private final GUICaptureReceiver client;
   
   private MP4MJPEGMovieBuilder builder;
   private PrintStream timestampStream;
   private int dts = 0;
   
   private volatile long timestamp = 0;
   
   public NetworkStreamVideoDataLogger(byte[] controlIP, File logPath, LogProperties logProperties, InetSocketAddress address) throws SocketException, UnknownHostException
   {
      super(logPath, logProperties, description);
      
      NetworkInterface iface = NetworkInterface.getByInetAddress(LogUtils.getMyIP(controlIP));
      
      
      client = new GUICaptureReceiver(iface, address.getAddress(), this);
      client.start();
   }

   @Override
   public void timestampChanged(long newTimestamp)
   {
      this.timestamp = newTimestamp;
   }

   @Override
   public void restart() throws IOException
   {
      try
      {
         if(builder != null)
         {
            builder.close();
         }
         if(timestampStream != null)
         {
            timestampStream.close();
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      builder = null;
      timestampStream = null;
      removeLogFiles();
   }

   @Override
   public void close()
   {
      client.close();
      try
      {
         if(builder != null)
         {
            builder.close();
         }
         if(timestampStream != null)
         {
            timestampStream.close();
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      builder = null;
      timestampStream = null;
   }
   
   @Override
   public void timestampReceived(long timestamp)
   {
      
   }

   @Override
   public void newDataAvailable(LogDataHeader header, ByteBuffer buffer)
   {
      if(builder == null)
      {
         try
         {
            // Decode the first image using ImageIO, to get the dimensions
            ByteBufferInputStream is = new ByteBufferInputStream();
            is.setByteBuffer(buffer);
            BufferedImage img = ImageIO.read(is);
            if(img == null)
            {
               System.err.println("Cannot decode image");
               return;
            }
            File videoFileFile = new File(videoFile);
            builder = new MP4MJPEGMovieBuilder(videoFileFile, img.getWidth(), img.getHeight(), 10, 90);
            File timestampFile = new File(timestampData);
            timestampStream = new PrintStream(timestampFile);
            timestampStream.println("1");
            timestampStream.println("10");
            buffer.clear();
            
            dts = 0;
         }
         catch (IOException e)
         {
            e.printStackTrace();
            return;
         }
         
         
      }
      try
      {
         builder.encodeFrame(buffer);
         timestampStream.println(timestamp + " " +  dts);
         dts++;
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   @Override
   public void timeout()
   {
      
   }

   @Override
   public void connected(InetSocketAddress localAddress)
   {
   }

   @Override
   public void keepAlive()
   {
      
   }

}
