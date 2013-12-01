package us.ihmc.graphics3DAdapter.camera;

import us.ihmc.graphics3DAdapter.camera.VideoSettings.VideoCompressionKey;

public interface VideoControlSettings
{
   public boolean isSendVideo();
   public VideoCompressionKey getVideoQualitySetting();
   public int getFps();
}