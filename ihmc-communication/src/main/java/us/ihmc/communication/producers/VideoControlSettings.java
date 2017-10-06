package us.ihmc.communication.producers;




public interface VideoControlSettings
{
   public boolean isSendVideo();
   public boolean crop();
   
   
   public int getBandwidthInKbit();
   public int getHorizontalResolution();
   public int getFps();
   
   public int cropX();
   public int cropY();
}