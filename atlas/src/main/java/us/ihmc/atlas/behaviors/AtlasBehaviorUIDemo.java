package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.apache.commons.collections4.ListUtils;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.FakeREAModule;
import us.ihmc.humanoidBehaviors.tools.FlatGroundPlanarRegionPublisher;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.concurrent.TimeUnit;

/**
 * Runs self contained behavior demo.
 */
public class AtlasBehaviorUIDemo extends Application
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean USE_FLAT_GROUND = false;

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      if (!USE_FLAT_GROUND)
      {
         new Thread(() -> {
            LogTools.info("Creating planar region publisher");
            new FakeREAModule(createPlanarRegions());
         }).start();

         new Thread(() -> {
            LogTools.info("Creating bipedal support region publisher");
            new BipedalSupportPlanarRegionPublisher(createRobotModel()).start();
         }).start();

         new Thread(() -> {
            LogTools.info("Creating flat ground region publisher");
            new FlatGroundPlanarRegionPublisher();
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
      PlanarRegionsListExamples.generateCinderBlockField(generator,
                                                         0.4,
                                                         0.1,
                                                         5,
                                                         6,
                                                         0.02,
                                                         -0.03,
                                                         1.5,
                                                         0.0,
                                                         Math.toRadians(15.0),
                                                         Math.toRadians(15.0),
                                                         0.05,
                                                         false);
      return generator.getPlanarRegionsList();
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values,
                                                                                                     8,
                                                                                                     3,
                                                                                                     true,
                                                                                                     true);
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);

   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
