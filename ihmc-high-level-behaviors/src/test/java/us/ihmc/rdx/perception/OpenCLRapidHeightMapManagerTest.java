package us.ihmc.rdx.perception;

public class OpenCLRapidHeightMapManagerTest extends RapidHeightMapManagerTest
{
   @Override
   public boolean runUsingCUDA()
   {
      return false;
   }
}
