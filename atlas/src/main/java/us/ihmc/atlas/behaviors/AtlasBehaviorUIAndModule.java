package us.ihmc.atlas.behaviors;

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

public class AtlasBehaviorUIAndModule
{
   private BehaviorUIRegistry behaviorRegistry;

   public AtlasBehaviorUIAndModule(BehaviorUIRegistry behaviorRegistry)
   {
      this.behaviorRegistry = behaviorRegistry;

      CommunicationMode ros2CommunicationMode = CommunicationMode.INTERPROCESS;
      CommunicationMode behaviorMessagerCommunicationMode = CommunicationMode.INTRAPROCESS;
      start(createRobotModel(), ros2CommunicationMode, behaviorMessagerCommunicationMode);
   }

   public void start(DRCRobotModel robotModel, CommunicationMode ros2CommunicationMode, CommunicationMode behaviorMessagerCommunicationMode)
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

      LogTools.info("Starting behavior module");
      BehaviorModule behaviorModule = new BehaviorModule(behaviorRegistry, robotModel, ros2CommunicationMode, behaviorMessagerCommunicationMode);

      LogTools.info("Starting behavior UI");
      BehaviorUI behaviorUI = BehaviorUI.create(behaviorRegistry,
                                                robotModel,
                                                ros2CommunicationMode,
                                                behaviorMessagerCommunicationMode,
                                                "localhost",
                                                behaviorModule.getMessager());
   }
   
   private DRCRobotModel createRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
   }

   public static void main(String[] args)
   {
      new AtlasBehaviorUIAndModule(BehaviorUIRegistry.DEFAULT_BEHAVIORS);
   }
}
