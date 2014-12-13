package us.ihmc.communication.producers;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.openh264.RC_MODES;
import org.openh264.SEncParamExt;

import us.ihmc.codecs.h264.NALProcessor;
import us.ihmc.codecs.h264.NALType;
import us.ihmc.codecs.h264.OpenH264Encoder;
import us.ihmc.codecs.yuv.YUV420Picture;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.sensing.VideoControlPacket;
import us.ihmc.utilities.VideoDataServer;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.TimeTools;

import com.google.code.libyuv.FilterModeEnum;

public class CompressedVideoDataServer implements NetStateListener, VideoDataServer, ObjectConsumer<VideoControlPacket>
{
   private OpenH264Encoder encoder;

   
   private boolean videoEnabled = false;
   private int desiredHorizontalResolution = 320;
   private int desiredBandwidth = 10;
   private int desiredFPS = 1;
   private int fps = 0;
   
   private int horizontalResolution = desiredHorizontalResolution;
   private int verticalResolution = desiredHorizontalResolution;
   
   private int bandwidth = desiredBandwidth;
   
   private boolean cropVideo = false;
   private int cropX;
   private int cropY;

   
   private final CompressedVideoHandler handler;

   private long initialTimestamp;
   private long prevTimeStamp;

   public int getFps()
   {
      return fps;
   }

   public CompressedVideoDataServer(CompressedVideoHandler handler)
   {
      this.handler = handler;
      
      this.handler.addNetStateListener(this);
      
      try
      {
         encoder = new OpenH264Encoder();
         SEncParamExt param = encoder.createParamExt(horizontalResolution, verticalResolution, bandwidth * 1024, RC_MODES.RC_LOW_BW_MODE);
         param.setFMaxFrameRate(25);
         param.setUiIntraPeriod(-1);
         encoder.initialize(param);
         
         
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      
   }

   public synchronized void updateImage(BufferedImage bufferedImage, final long timeStamp, final Point3d cameraPosition, final Quat4d cameraOrientation, final double fov)
   {
      if(!handler.isConnected() || !videoEnabled)
      {
         return;
      }
      if(desiredFPS != fps)
      {
         encoder.setFrameRate(desiredFPS);
         fps = desiredFPS;
      }
      
      if (initialTimestamp == -1)
      {
         initialTimestamp = timeStamp;
      }
      else if((timeStamp - prevTimeStamp) < TimeTools.secondsToNanoSeconds(1.0/((double)fps)))
      {
         return;
      }
      
      if(desiredBandwidth != bandwidth)
      {
         encoder.setTargetBitRate(desiredBandwidth * 1024);
         bandwidth = desiredBandwidth;
      }
            
      int desiredEvenHorizontalResolution = (desiredHorizontalResolution >> 1) << 1;
      int desiredVerticalResolution = (bufferedImage.getHeight() * desiredEvenHorizontalResolution) / bufferedImage.getWidth();
      int desiredEvenVerticalResolution = (desiredVerticalResolution >> 1) << 1;
      
      
      if(desiredEvenHorizontalResolution != horizontalResolution || desiredEvenVerticalResolution != verticalResolution)
      {
         encoder.setResolution(desiredEvenHorizontalResolution, desiredEvenVerticalResolution);
         horizontalResolution = desiredEvenHorizontalResolution;
         verticalResolution = desiredEvenVerticalResolution;
      }
      
      if(cropVideo)
      {
         int width = bufferedImage.getWidth();
         int height = bufferedImage.getHeight();
         
         int newWidth = width/2;
         int newHeight = height/2;
         
         int x = (newWidth * cropX)/100;
         int y = (newHeight * cropY)/100;
         
         BufferedImage croppedImage = new BufferedImage(newWidth, newHeight, bufferedImage.getType());
         Graphics2D graphics = croppedImage.createGraphics();
         graphics.drawImage(bufferedImage, 0, 0, newWidth, newHeight, x, y, x + newWidth, y + newHeight, null);
         graphics.dispose();
         bufferedImage = croppedImage;
      }

      YUV420Picture frame = new YUV420Picture(bufferedImage);
      YUV420Picture scaled = frame.scale(desiredEvenHorizontalResolution, desiredEvenVerticalResolution, FilterModeEnum.kFilterBilinear);
      try
      {
         encoder.encodeFrame(scaled, new NALProcessor()
         {
            
            @Override
            public void processNal(NALType type, ByteBuffer buffer) throws IOException
            {
               byte[] data = new byte[buffer.remaining()];
               buffer.get(data);
               handler.newVideoPacketAvailable(timeStamp, data, cameraPosition, cameraOrientation, fov);
            }
         });
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      finally
      {
         frame.delete();
         scaled.delete();
      }
      
     
      
      prevTimeStamp = timeStamp;
   }
   
   


   public synchronized void close()
   {
      encoder.delete();
   }
   
   public boolean isConnected()
   {
      return handler.isConnected();
   }

   public synchronized void connected()
   {
      encoder.sendIntraFrame();
   }

   public synchronized void disconnected()
   {
      videoEnabled = false;
   }


   public synchronized void setVideoControlSettings(VideoControlSettings object)
   {
      if(object.isSendVideo())
      {
         desiredHorizontalResolution = object.getHorizontalResolution();
         desiredBandwidth = object.getBandwidthInKbit();
         
         desiredFPS = object.getFps();
         cropVideo = object.crop();
         videoEnabled = true;
         
         cropX = MathTools.clipToMinMax(object.cropX(), 0, 100);
         cropY = MathTools.clipToMinMax(object.cropY(), 0, 100);
         
      }
      else
      {
         videoEnabled = false;
      }
   }

   public void consumeObject(VideoControlPacket object)
   {
      setVideoControlSettings(object);
   }

}