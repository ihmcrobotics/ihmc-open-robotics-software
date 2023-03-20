package us.ihmc.missionControl.resourceMonitor;

public class FreeMemoryMonitor extends ResourceMonitor
{
   private float memoryTotalGiB;
   private float memoryUsedGiB;

   public FreeMemoryMonitor()
   {
      super("free", "--mebi");
   }

   public float getMemoryTotalGiB()
   {
      return memoryTotalGiB;
   }

   public float getMemoryUsedGiB()
   {
      return memoryUsedGiB;
   }

   @Override
   public void parse(String[] lines)
   {
      if (lines.length == 0) return;
      String[] amongSpaces = lines[1].split("\\s+");
      memoryTotalGiB = (float) (Float.parseFloat(amongSpaces[1]) / 1000.0);
      memoryUsedGiB = (float) (Float.parseFloat(amongSpaces[2]) / 1000.0);
   }
}
