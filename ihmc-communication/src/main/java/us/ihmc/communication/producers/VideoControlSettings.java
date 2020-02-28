package us.ihmc.communication.producers;

public class VideoControlSettings
{
   private int qualityFactor = 75;
   private boolean isSendVideo = true;
   private boolean crop = false;
   private int cropX = 0;
   private int cropY = 0;
   private int bandwidthInKbit;
   private int horizontalResolution;
   private int fps = 25;

   public VideoControlSettings()
   {
   }

   public static VideoControlSettings configureJPEGServer(int qualityFactor, int fps)
   {
      VideoControlSettings settings = new VideoControlSettings();
      settings.setSendVideo(true);
      settings.setCrop(false);
      settings.setQualityFactor(qualityFactor);
      settings.setFPS(fps);
      return settings;
   }

   public void setQualityFactor(int qualityFactor)
   {
      this.qualityFactor = qualityFactor;
   }

   public void setSendVideo(boolean isSendVideo)
   {
      this.isSendVideo = isSendVideo;
   }

   public void setCrop(boolean crop)
   {
      this.crop = crop;
   }

   public void setCropX(int cropX)
   {
      this.cropX = cropX;
   }

   public void setCropY(int cropY)
   {
      this.cropY = cropY;
   }

   public void setBandwidthInKbit(int bandwidthInKbit)
   {
      this.bandwidthInKbit = bandwidthInKbit;
   }

   public void setHorizontalResolution(int horizontalResolution)
   {
      this.horizontalResolution = horizontalResolution;
   }

   public void setFPS(int fps)
   {
      this.fps = fps;
   }

   /**
    * Used by the JPEG encoder only, quality should be in [0, 100] where 100 represents the best image
    * quality.
    */
   public int getQualityFactor()
   {
      return qualityFactor;
   }

   /**
    * Essentially for enabling/disabling the server.
    */
   public boolean isSendVideo()
   {
      return isSendVideo;
   }

   /**
    * Whether the video should be cropped before being sent. When {@code true}, {@link #getCropX()} and
    * {@link #getCropY()} are required.
    */
   public boolean isCropped()
   {
      return crop;
   }

   /**
    * Only used when {@link #isCropped()} returns {@code true}, the value should be in [0, 100]
    * representing the width percentage of the image that should be cropped.
    */
   public int getCropX()
   {
      return cropX;
   }

   /**
    * Only used when {@link #isCropped()} returns {@code true}, the value should be in [0, 100]
    * representing the height percentage of the image that should be cropped.
    */
   public int getCropY()
   {
      return cropY;
   }

   public int getBandwidthInKbit()
   {
      return bandwidthInKbit;
   }

   public int getHorizontalResolution()
   {
      return horizontalResolution;
   }

   /**
    * Gets the desired rate at which the image should be published.
    */
   public int getFPS()
   {
      return fps;
   }
}