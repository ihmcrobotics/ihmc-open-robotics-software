package us.ihmc.robotDataLogger.logger;

public enum VideoIn
{
   COMPOSITE(1),
   COMPONENT(2),
   HDMI(3),
   SDI(4),
   OPTICAL_SDI(5),
   S_VIDEO(6);
   
   
   public int command;
   private VideoIn(int command)
   {
      this.command = command;
   }
}