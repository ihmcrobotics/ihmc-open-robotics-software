package us.ihmc.rdx.ui.processes;

import us.ihmc.behaviors.tools.perception.DockerMapSense;

public class MapSenseHeadlessProcess extends RestartableProcess
{
   private DockerMapSense mapsense;

   @Override
   protected void startInternal()
   {
      mapsense = new DockerMapSense();
      mapsense.start();
   }

   @Override
   protected void stopInternal()
   {
      mapsense.shutdown();
      mapsense = null;
   }

   @Override
   public String getName()
   {
      return "MapSense (headless)";
   }
}
