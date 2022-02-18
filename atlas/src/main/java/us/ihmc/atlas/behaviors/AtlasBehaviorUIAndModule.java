package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUI;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class AtlasBehaviorUIAndModule
{
   private JavaFXBehaviorUIRegistry behaviorRegistry;

   public AtlasBehaviorUIAndModule(JavaFXBehaviorUIRegistry behaviorRegistry)
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
      boolean enableROS1 = true;
      BehaviorModule behaviorModule = new BehaviorModule(behaviorRegistry, robotModel, ros2CommunicationMode, behaviorMessagerCommunicationMode, enableROS1);

      LogTools.info("Starting behavior UI");
      JavaFXBehaviorUI behaviorUI = JavaFXBehaviorUI.create(behaviorRegistry,
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
      new AtlasBehaviorUIAndModule(JavaFXBehaviorUIRegistry.DEFAULT_BEHAVIORS);
   }
}
