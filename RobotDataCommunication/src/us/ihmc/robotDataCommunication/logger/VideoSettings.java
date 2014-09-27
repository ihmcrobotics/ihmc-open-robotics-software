package us.ihmc.robotDataCommunication.logger;

import us.ihmc.robotDataCommunication.logger.util.BMDCapture.VideoIn;

public enum VideoSettings
{
/**
 * 
 * Known BlackMagic Mini Recorder modes
 * 
 * mode = 11 -> 1080i59.94Hz
 * mode = 12 -> 1080i60Hz
 * mode = 9 -> 1080p30
 * mode = 14 -> 720p59.94Hz
 * mode = 15 -> 720p60Hz
*/   
   
   SONY_720P60_TRIPOD("Tripod", 0, 14, false, VideoIn.SDI),
   BLACKMAGIC_1080P30_CRANE("AFrame", 1, 9, false, VideoIn.SDI),
   BLACKMAGIC_1080P30_TRIPOD("Tripod", 0, 9, false, VideoIn.SDI),
   CANON_XA25_720P("Tripod", 0, 14, false, VideoIn.SDI),
   MARSHALL_CV330_720P("GorillaPod", 1, 15, false, VideoIn.SDI),
   MARSHALL_CV330_1080P("GorillaPod", 1, 9, false, VideoIn.SDI),
   SANDIA_CV330_0("Sandia1", 0, 9, false, VideoIn.SDI),
   SANDIA_CV330_1("Sandia2", 1, 9, false, VideoIn.SDI);
   
   
   
   private final String description;
   private final int device;
   private final int mode;
   private final boolean interlaced;
   private final VideoIn videoIn;

   
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
   
   private VideoSettings(String description, int device, int mode, boolean interlaced, VideoIn videoIn)
   {
      this.description = description;
      this.device = device;
      this.mode = mode;
      this.interlaced = interlaced;
      this.videoIn = videoIn;
   }

   public VideoIn getVideoIn()
   {
      return videoIn;
   }

}
