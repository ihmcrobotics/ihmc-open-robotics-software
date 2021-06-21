package us.ihmc.gdx.ui.missionControl.processes;

import us.ihmc.communication.ros.DockerROS1Master;
import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;

public class ROS1MasterProcess extends RestartableMissionControlProcess
{
   private DockerROS1Master ros1Master;

   @Override
   protected void startInternal()
   {
      ros1Master = new DockerROS1Master();
      ros1Master.start();
   }

   @Override
   protected void stopInternal()
   {
      ros1Master.shutdown();
      ros1Master = null;
   }

   @Override
   public String getName()
   {
      return "ROS 1 Master";
   }
}
