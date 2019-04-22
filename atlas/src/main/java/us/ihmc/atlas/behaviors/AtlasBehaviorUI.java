package us.ihmc.atlas.behaviors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.parameterTuner.remote.ParameterTuner;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public class AtlasBehaviorUI extends Application
{
   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);

      Messager behaviorMessager = RemoteBehaviorInterface.createForUI(NetworkParameters.getHost(NetworkParameterKeys.networkManager));

      ui = new BehaviorUI(primaryStage,
                          behaviorMessager,
                          drcRobotModel,
                          PubSubImplementation.FAST_RTPS);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      new Thread(() -> {
         LogTools.info("Spawning parameter tuner");
         new JavaProcessSpawner(true).spawn(ParameterTuner.class); // NPE if ParameterTuner started in same process, so spawn it
      }).start();

      launch(args);
   }
}
