package us.ihmc.graphics3DAdapter.camera;

import us.ihmc.graphics3DAdapter.camera.VideoSettings.VideoCompressionKey;

public interface VideoControlSettings
{
   public boolean isSendVideo();
   public boolean crop();
   public VideoCompressionKey getVideoQualitySetting();
   public int getFps();
   
   public int cropX();
   public int cropY();
}