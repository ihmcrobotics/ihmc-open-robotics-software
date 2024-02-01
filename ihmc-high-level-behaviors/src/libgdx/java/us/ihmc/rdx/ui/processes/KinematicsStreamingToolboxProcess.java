package us.ihmc.rdx.ui.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

import java.util.Map;

public class KinematicsStreamingToolboxProcess extends RestartableProcess
{
   private final DRCRobotModel robotModel;
   private Map<String, Double> initialConfiguration;
   private boolean startYoVariableServer;
   private KinematicsStreamingToolboxModule kinematicsStreamingToolboxModule;

   public KinematicsStreamingToolboxProcess(DRCRobotModel robotModel, Map<String, Double> initialConfiguration, boolean startYoVariableServer)
   {
      this.robotModel = robotModel;
      this.initialConfiguration = initialConfiguration;
      this.startYoVariableServer = startYoVariableServer;
   }

   @Override
   protected void startInternal()
   {
      kinematicsStreamingToolboxModule = new KinematicsStreamingToolboxModule(robotModel, startYoVariableServer, PubSubImplementation.FAST_RTPS);
      KinematicsStreamingToolboxController controller = (KinematicsStreamingToolboxController) kinematicsStreamingToolboxModule.getToolboxController();
      controller.setInitialRobotConfigurationNamedMap(initialConfiguration);
      controller.getTools().getIKController().getCenterOfMassSafeMargin().set(0.10);
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
