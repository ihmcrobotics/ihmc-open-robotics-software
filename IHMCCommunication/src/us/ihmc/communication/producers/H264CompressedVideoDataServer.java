package us.ihmc.communication.producers;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.codecs.generated.EUsageType;
import us.ihmc.codecs.generated.FilterModeEnum;
import us.ihmc.codecs.generated.RC_MODES;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.h264.OpenH264Encoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.robotics.MathTools;

public class H264CompressedVideoDataServer implements NetStateListener, CompressedVideoDataServer
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
   
   private final YUVPictureConverter converter = new YUVPictureConverter();

   public int getFps()
   {
      return fps;
   }

   H264CompressedVideoDataServer(CompressedVideoHandler handler)
   {
      this.handler = handler;

      this.handler.addNetStateListener(this);

      encoder = new OpenH264Encoder();
      encoder.setIntraPeriod(-1);
      encoder.setRCMode(RC_MODES.RC_BITRATE_MODE);
      encoder.initialize(horizontalResolution, verticalResolution, 25.0, bandwidth * 1024, EUsageType.CAMERA_VIDEO_REAL_TIME);

   }

   @Override
   public synchronized void updateImage(VideoSource videoSource, BufferedImage bufferedImage, final long timeStamp, final Point3d cameraPosition, final Quat4d cameraOrientation,
         IntrinsicParameters intrinsicParameters)
   {
      if (!handler.isConnected() || !videoEnabled)
      {
         return;
      }
      if (desiredFPS != fps)
      {
         encoder.setMaxFrameRate(desiredFPS);
         fps = desiredFPS;
      }

      if (initialTimestamp == -1)
      {
         initialTimestamp = timeStamp;
      }
      else if ((timeStamp - prevTimeStamp) < Conversions.secondsToNanoSeconds(1.0 / ((double) fps)))
      {
         return;
      }

      if (desiredBandwidth != bandwidth)
      {
         encoder.setBitRate(desiredBandwidth * 1024);
         bandwidth = desiredBandwidth;
      }

      int desiredEvenHorizontalResolution = (desiredHorizontalResolution >> 1) << 1;
      int desiredVerticalResolution = (bufferedImage.getHeight() * desiredEvenHorizontalResolution) / bufferedImage.getWidth();
      int desiredEvenVerticalResolution = (desiredVerticalResolution >> 1) << 1;

      if (desiredEvenHorizontalResolution != horizontalResolution || desiredEvenVerticalResolution != verticalResolution)
      {
         encoder.setSize(desiredEvenHorizontalResolution, desiredEvenVerticalResolution);
         horizontalResolution = desiredEvenHorizontalResolution;
         verticalResolution = desiredEvenVerticalResolution;
      }

      if (cropVideo)
      {
         int width = bufferedImage.getWidth();
         int height = bufferedImage.getHeight();

         int newWidth = width / 2;
         int newHeight = height / 2;

         int x = (newWidth * cropX) / 100;
         int y = (newHeight * cropY) / 100;

         BufferedImage croppedImage = new BufferedImage(newWidth, newHeight, bufferedImage.getType());
         Graphics2D graphics = croppedImage.createGraphics();
         graphics.drawImage(bufferedImage, 0, 0, newWidth, newHeight, x, y, x + newWidth, y + newHeight, null);
         graphics.dispose();
         bufferedImage = croppedImage;
      }

      YUVPicture frame = converter.fromBufferedImage(bufferedImage, YUVSubsamplingType.YUV420);
      frame.scale(desiredEvenHorizontalResolution, desiredEvenVerticalResolution, FilterModeEnum.kFilterBilinear);
      try
      {
         encoder.encodeFrame(frame);
         frame.delete();
         while(encoder.nextNAL())
         {
            ByteBuffer nal = encoder.getNAL();
            byte[] data = new byte[nal.remaining()];
            nal.get(data);
            handler.newVideoPacketAvailable(videoSource, timeStamp, data, cameraPosition, cameraOrientation, intrinsicParameters);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
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
      if (object.isSendVideo())
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
}