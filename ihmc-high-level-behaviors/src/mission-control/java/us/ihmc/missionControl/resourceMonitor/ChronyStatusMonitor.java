package us.ihmc.missionControl.resourceMonitor;

public class ChronyStatusMonitor extends ResourceMonitor
{
   private static final String NOT_SYNCED_MESSAGE = "chrony is not currently synchronized with a source";

   private String chronyBestSource;

   public ChronyStatusMonitor()
   {
      super("chronyc", "sources");
      chronyBestSource = "chronyc not yet parsed";
   }

   public String getChronyBestSource()
   {
      return chronyBestSource;
   }

   @Override
   public void parse(String[] lines)
   {
      String chronyBestSource = NOT_SYNCED_MESSAGE;
      for (String line : lines)
      {
         if (line.contains("*")) // This is the selected best source by chrony.
         {
            chronyBestSource = line;
            break;
         }
      }
      this.chronyBestSource = chronyBestSource;
   }
}
