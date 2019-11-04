package us.ihmc.atlas.behaviors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasBehaviorUIAndModule extends Application
{
   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);

      Messager behaviorMessager = RemoteBehaviorInterface.createForUI("localhost");

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
      ThreadTools.startAThread(() -> new AtlasBehaviorModule(), AtlasBehaviorUIAndModule.class.getSimpleName());
      launch(args);
   }
}
