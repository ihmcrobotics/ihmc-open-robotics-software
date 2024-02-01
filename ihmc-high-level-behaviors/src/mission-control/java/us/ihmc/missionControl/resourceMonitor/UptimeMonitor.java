package us.ihmc.missionControl.resourceMonitor;

public class UptimeMonitor extends ResourceMonitor
{
   private String uptime;

   public UptimeMonitor()
   {
      super("uptime");
   }

   public String getUptime()
   {
      return uptime;
   }

   @Override
   public void parse(String[] lines)
   {
      if (lines.length > 0)
         uptime = lines[0];
   }
}
