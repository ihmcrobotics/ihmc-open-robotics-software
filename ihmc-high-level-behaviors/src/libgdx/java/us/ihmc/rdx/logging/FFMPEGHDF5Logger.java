package us.ihmc.rdx.logging;

import us.ihmc.perception.BytedecoImage;

public class FFMPEGHDF5Logger extends FFMPEGLogger
{
   @Override
   public boolean put(BytedecoImage sourceImage)
   {
      return false;
   }

   public FFMPEGHDF5Logger(int sourceVideoWidth,
                           int sourceVideoHeight,
                           boolean lossless,
                           int framerate,
                           int bitRate,
                           int sourcePixelFormat,
                           int encoderPixelFormat,
                           String fileName,
                           String preferredVideoEncoder)
   {
      super(sourceVideoWidth, sourceVideoHeight, lossless, framerate, bitRate, sourcePixelFormat, encoderPixelFormat, fileName, preferredVideoEncoder);
   }
}
