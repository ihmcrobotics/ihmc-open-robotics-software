package us.ihmc.atlas.behaviors;

import com.esotericsoftware.minlog.Log;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.humanoidBehaviors.BehaviorBackpack;
import us.ihmc.humanoidBehaviors.BehaviorTeleop;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

/**
 * Bundles behavior module and footstep planning toolbox.
 */
public class AtlasBehaviorUIDemo extends Application
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Log.DEBUG();

      new Thread(() ->
         AtlasBehaviorSimulation.createForManualTest(new AtlasRobotModel(AtlasBehaviorBackpack.ATLAS_VERSION, RobotTarget.SCS, false),
                                                     new FlatGroundEnvironment())
                                .simulate()
      ).start();

      new Thread(() -> {
         AtlasRobotModel robotModel = new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
         LogTools.info("Creating footstep toolbox");
         new MultiStageFootstepPlanningModule(robotModel, null, false, DomainFactory.PubSubImplementation.FAST_RTPS);
      }).start();

      new Thread(() -> {
         AtlasRobotModel robotModel = new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
         LogTools.info("Creating behavior backpack");
         BehaviorBackpack.createForBackpack(robotModel);
      }).start();

      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);

      BehaviorTeleop teleop = BehaviorTeleop.createForUI(robotModel, NetworkParameters.getHost(NetworkParameterKeys.networkManager));

      ui = new BehaviorUI(primaryStage,
                          teleop,
                          robotModel,
                          PubSubImplementation.FAST_RTPS);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      ui.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
