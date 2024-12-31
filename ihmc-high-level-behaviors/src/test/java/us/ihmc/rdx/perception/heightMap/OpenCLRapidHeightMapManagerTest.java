package us.ihmc.rdx.perception.heightMap;

public class OpenCLRapidHeightMapManagerTest extends RapidHeightMapManagerTest
{
   @Override
   public boolean runUsingCUDA()
   {
      return false;
   }
}
