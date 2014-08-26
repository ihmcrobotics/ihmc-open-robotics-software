package us.ihmc.graphics3DAdapter.camera;



public interface VideoControlSettings
{
   public boolean isSendVideo();
   public boolean crop();
   public VideoCompressionKey getVideoQualitySetting();
   public int getFps();
   
   public int cropX();
   public int cropY();
}