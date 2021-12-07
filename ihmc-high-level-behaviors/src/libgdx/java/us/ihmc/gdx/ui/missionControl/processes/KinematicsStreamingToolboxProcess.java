package us.ihmc.gdx.ui.missionControl.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class KinematicsStreamingToolboxProcess extends RestartableMissionControlProcess
{
   private final DRCRobotModel robotModel;
   private KinematicsStreamingToolboxModule kinematicsStreamingToolboxModule;

   public KinematicsStreamingToolboxProcess(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   @Override
   protected void startInternal()
   {
      kinematicsStreamingToolboxModule = new KinematicsStreamingToolboxModule(robotModel, false, PubSubImplementation.FAST_RTPS);
      kinematicsStreamingToolboxModule.wakeUp();
   }

   @Override
   protected void stopInternal()
   {
      kinematicsStreamingToolboxModule.destroy();
   }

   @Override
   public String getName()
   {
      return "Kinematics Streaming Toolbox";
   }

   public KinematicsStreamingToolboxModule getKinematicsStreamingToolboxModule()
   {
      return kinematicsStreamingToolboxModule;
   }
}
