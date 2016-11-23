package us.ihmc.tools.time;

public class CpuTimeProvider implements TimeProvider
{
   @Override
   public double now()
   {
      return System.nanoTime() / 1e9;
   }
}
