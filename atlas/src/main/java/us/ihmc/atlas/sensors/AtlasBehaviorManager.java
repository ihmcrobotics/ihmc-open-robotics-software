package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;

public class AtlasBehaviorManager
{
   private IHMCHumanoidBehaviorManager behaviorManager;

   public AtlasBehaviorManager()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      AtlasSensorInformation sensorInformation = (AtlasSensorInformation) robotModel.getSensorInformation();

      ExceptionTools.handle(() ->
      {
         LogModelProvider logModelProvider = robotModel.getLogModelProvider();
         boolean startYoVariableServer = false;
         behaviorManager = new IHMCHumanoidBehaviorManager(robotModel.getSimpleRobotName(),
                                                           robotModel.getFootstepPlannerParameters(),
                                                           robotModel,
                                                           robotModel,
                                                           logModelProvider,
                                                           startYoVariableServer,
                                                           sensorInformation);
      }, DefaultExceptionHandler.RUNTIME_EXCEPTION);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down network processor modules.");
         if (behaviorManager != null)
            behaviorManager.closeAndDispose();
         ThreadTools.sleep(10);
      }, getClass().getSimpleName() + "Shutdown"));
   }

   public static void main(String[] args)
   {
      new AtlasBehaviorManager();
   }
}
