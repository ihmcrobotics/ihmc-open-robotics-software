package us.ihmc.robotDataCommunication.logger;

public enum VideoSettings
{
/**
 * 
 * Known BlackMagic Mini Recorder modes
 * 
 * mode = 11 -> 1080p59.94Hz
 * mode = 12 -> 1080p60Hz
 * mode = 9 -> 1080p60
 * mode = 14 -> 720p59.94Hz
*/   
   
   SONY_720P60_TRIPOD("Tripod", 0, 14, false),
   BLACKMAGIC_1080P30_CRANE("Crane", 1, 9, false);
   
   
   
   private final String description;
   private final int device;
   private final int mode;
   private final boolean interlaced;


   
   public String getDescription()
   {
      return description;
   }
   
   public int getDevice()
   {
      return device;
   }
   
   public int getMode()
   {
      return mode;
   }
   
   public boolean isInterlaced()
   {
      return interlaced;
   }
   
   private VideoSettings(String description, int device, int mode, boolean interlaced)
   {
      this.description = description;
      this.device = device;
      this.mode = mode;
      this.interlaced = interlaced;
   }

}
