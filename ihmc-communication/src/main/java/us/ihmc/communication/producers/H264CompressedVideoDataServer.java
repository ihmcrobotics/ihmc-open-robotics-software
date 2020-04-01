package us.ihmc.communication.producers;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.codecs.generated.EUsageType;
import us.ihmc.codecs.generated.FilterModeEnum;
import us.ihmc.codecs.generated.RC_MODES;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.h264.OpenH264Encoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.commons.MathTools;

public class H264CompressedVideoDataServer implements ConnectionStateListener, CompressedVideoDataServer
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

   private long previousTimestamp = -1;

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
   public synchronized void onFrame(VideoSource videoSource, BufferedImage bufferedImage, final long timeStamp, final Point3DReadOnly cameraPosition,
                                    final QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParameters)
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

      if (previousTimestamp > -1 && (timeStamp - previousTimestamp) < Conversions.secondsToNanoseconds(1.0 / ((double) fps)))
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
         while (encoder.nextNAL())
         {
            ByteBuffer nal = encoder.getNAL();
            byte[] data = new byte[nal.remaining()];
            nal.get(data);
            handler.onFrame(videoSource, data, timeStamp, cameraPosition, cameraOrientation, intrinsicParameters);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      previousTimestamp = timeStamp;
   }

   @Override
   public synchronized void dispose()
   {
      encoder.delete();
   }

   @Override
   public boolean isConnected()
   {
      return handler.isConnected();
   }

   @Override
   public synchronized void connected()
   {
      encoder.sendIntraFrame();
   }

   @Override
   public synchronized void disconnected()
   {
      videoEnabled = false;
   }

   @Override
   public synchronized void setVideoControlSettings(VideoControlSettings object)
   {
      if (object.isSendVideo())
      {
         desiredHorizontalResolution = object.getHorizontalResolution();
         desiredBandwidth = object.getBandwidthInKbit();

         desiredFPS = object.getFPS();
         cropVideo = object.isCropped();
         videoEnabled = true;

         cropX = MathTools.clamp(object.getCropX(), 0, 100);
         cropY = MathTools.clamp(object.getCropY(), 0, 100);

      }
      else
      {
         videoEnabled = false;
      }
   }
}