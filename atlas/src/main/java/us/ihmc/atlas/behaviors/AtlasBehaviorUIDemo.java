package us.ihmc.atlas.behaviors;

import com.esotericsoftware.minlog.Log;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.util.concurrent.TimeUnit;

/**
 * Runs self contained behavior demo.
 */
public class AtlasBehaviorUIDemo extends Application
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean USE_FLAT_GROUND = true;

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Log.DEBUG();

      if (!USE_FLAT_GROUND)
      {
         new Thread(() -> {
            LogTools.info("Creating planar region publisher");
            Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "Fake_REA_module");
            IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher
                  = ROS2Tools.createPublisher(ros2Node,
                                              PlanarRegionsListMessage.class,
                                              ROS2Tools.getTopicNameGenerator(createRobotModel().getSimpleRobotName(),
                                                                              ROS2Tools.REA_MODULE,
                                                                              ROS2Tools.ROS2TopicQualifier.OUTPUT));
            PlanarRegionsList planarRegions = createPlanarRegions();
            PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegions);
            PeriodicNonRealtimeThreadScheduler patrolThread = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
            patrolThread.schedule(() -> planarRegionPublisher.publish(planarRegionsListMessage), 500, TimeUnit.MILLISECONDS);
         }).start();
      }

      new Thread(() -> {
         LogTools.info("Creating simulation");
         AtlasBehaviorSimulation.createForManualTest(createRobotModel(),
                                                     USE_FLAT_GROUND ?
                                                     new FlatGroundEnvironment() :
                                                     new PlanarRegionsListDefinedEnvironment(createPlanarRegions(),
                                                                                             0.02,
                                                                                             false)
         )
                                .simulate();
      }
      ).start();

      new Thread(() -> {
         LogTools.info("Creating footstep toolbox");
         new MultiStageFootstepPlanningModule(createRobotModel(), null, false, DomainFactory.PubSubImplementation.FAST_RTPS);
      }).start();

      new Thread(() -> {
         LogTools.info("Creating behavior backpack");
         BehaviorModule.createForBackpack(createRobotModel());
      }).start();

      LogTools.info("Creating behavior user interface");
      AtlasRobotModel robotModel = createRobotModel();
      Messager behaviorMessager = RemoteBehaviorInterface.createForUI("localhost");
      ui = new BehaviorUI(primaryStage, behaviorMessager, robotModel, PubSubImplementation.FAST_RTPS);
      ui.show();
   }

   private PlanarRegionsList createPlanarRegions()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      double startingBlockLength = 1.0;
      double cinderBlockSize = 0.4;
      double cinderBlockHeight = 0.10;
      double courseLength = 2.0;
      double courseWidth = 1.5;
      double heightVariation = 0.05;
      double extrusionLength = -0.05;
      double percentageAbsent = 0.0;
      double minTilt = Math.toRadians(0.0);
      double maxTilt = Math.toRadians(15.0);
      double randomHeightVariation = 0.0;
      PlanarRegionsListExamples.generateCinderBlockField(generator, cinderBlockSize, cinderBlockHeight, (int) Math.round(courseLength / cinderBlockSize),
                                                         (int) Math.round(courseWidth / cinderBlockSize), heightVariation, extrusionLength,
                                                         startingBlockLength, percentageAbsent, minTilt, maxTilt, randomHeightVariation);
      return generator.getPlanarRegionsList();
   }

   private AtlasRobotModel createRobotModel()
   {
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
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
