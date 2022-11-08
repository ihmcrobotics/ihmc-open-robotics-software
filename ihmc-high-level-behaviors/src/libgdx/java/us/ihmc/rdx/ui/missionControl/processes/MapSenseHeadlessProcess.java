package us.ihmc.rdx.ui.missionControl.processes;

import us.ihmc.behaviors.tools.perception.DockerMapSense;
import us.ihmc.rdx.ui.missionControl.RestartableMissionControlProcess;

public class MapSenseHeadlessProcess extends RestartableMissionControlProcess
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
