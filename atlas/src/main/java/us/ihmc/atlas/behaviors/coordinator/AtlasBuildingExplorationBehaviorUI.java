package us.ihmc.atlas.behaviors.coordinator;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehavior;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class AtlasBuildingExplorationBehaviorUI
{
   public static void start()
   {
      start(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false),
            CommunicationMode.INTERPROCESS,
            CommunicationMode.INTRAPROCESS);
   }

   public static void start(DRCRobotModel robotModel, CommunicationMode ros2CommunicationMode, CommunicationMode behaviorMessagerCommunicationMode)
   {
      LogTools.info("Starting humanoid behavior manager");
      ExceptionTools.handle(() ->
      {
         HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();
         new IHMCHumanoidBehaviorManager(robotModel.getSimpleRobotName(),
                                         robotModel.getFootstepPlannerParameters(),
                                         robotModel,
                                         robotModel,
                                         logModelProvider,
                                         false,
                                         sensorInformation);
      }, DefaultExceptionHandler.RUNTIME_EXCEPTION);

      BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.DEFAULT_BEHAVIORS;

      LogTools.info("Starting behavior module");
      BehaviorModule behaviorModule = new BehaviorModule(behaviorRegistry, robotModel, ros2CommunicationMode, behaviorMessagerCommunicationMode);

      LogTools.info("Starting behavior UI");
      BehaviorUI behaviorUI = BehaviorUI.create(behaviorRegistry,
                                                robotModel,
                                                ros2CommunicationMode,
                                                behaviorMessagerCommunicationMode,
                                                "localhost",
                                                behaviorModule.getMessager());
      behaviorUI.selectBehavior(BuildingExplorationBehavior.DEFINITION);
   }

   public static void main(String[] args)
   {
      AtlasBuildingExplorationBehaviorUI.start();
   }
}
