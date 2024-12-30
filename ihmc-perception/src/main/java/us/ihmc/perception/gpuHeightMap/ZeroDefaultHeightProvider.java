package us.ihmc.perception.gpuHeightMap;

public class ZeroDefaultHeightProvider implements DefaultHeightProvider
{
   @Override
   public double computeDefaultHeight()
   {
      return 0;
   }
}
