package us.ihmc.atlas;

import us.ihmc.atlas.behaviors.AtlasPerceptionSimulation;
import us.ihmc.atlas.behaviors.AtlasPerceptionSimulation.Fidelity;
import us.ihmc.atlas.behaviors.tools.AtlasSimulationBasics;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.ExploreAreaBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.simulation.EnvironmentInitialSetup;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.PlannerTestEnvironments;

public class AtlasExploreAreaBehaviorDemo extends AtlasSimulationBasics
{
   private static boolean RUN_REALSENSE_SLAM = Boolean.parseBoolean(System.getProperty("run.realsense.slam", "true"));
   private static boolean RUN_LIDAR_REA = Boolean.parseBoolean(System.getProperty("run.lidar.rea", "true"));
   private static boolean SHOW_REALSENSE_SLAM_UIS = Boolean.parseBoolean(System.getProperty("show.realsense.slam.uis"));

   {
      environmentInitialSetups.add(new EnvironmentInitialSetup(PlannerTestEnvironments::getTrickCorridorWidened,
                                                               0.0, 0.0, 0.0, 0.0));
      selectEnvironment();
   }

   private BehaviorUI behaviorUI;
   private final BehaviorModule behaviorModule;
   private AtlasPerceptionSimulation perceptionStack;

   public AtlasExploreAreaBehaviorDemo()
   {
      ThreadTools.startAsDaemon(() -> perceptionStack = new AtlasPerceptionSimulation(COMMUNICATION_MODE_ROS2,
                                                                                      environmentInitialSetup.getPlanarRegionsSupplier().get(),
                                                                                      RUN_REALSENSE_SLAM,
                                                                                      SHOW_REALSENSE_SLAM_UIS,
                                                                                      RUN_LIDAR_REA,
                                                                                      createRobotModel(),
                                                                                      Fidelity.LOW),
                               "PerceptionStack");
      ThreadTools.startAsDaemon(simulation, "Simulation");

      FootstepPlanningModuleLauncher.createModule(createRobotModel(), COMMUNICATION_MODE_ROS2.getPubSubImplementation());

      if (RUN_LIDAR_AND_CAMERA_SIMULATION)
         ThreadTools.startAsDaemon(this::lidarAndCameraSimulator, "LidarAndCamera");

      BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.of(ExploreAreaBehaviorUI.DEFINITION, LookAndStepBehaviorUI.DEFINITION);

      behaviorModule = new BehaviorModule(behaviorRegistry, createRobotModel(), COMMUNICATION_MODE_ROS2, COMMUNICATION_MODE_KRYO);

      LogTools.info("Creating behavior user interface");
      behaviorUI = BehaviorUI.create(behaviorRegistry,
                                     createRobotModel(),
                                     COMMUNICATION_MODE_ROS2,
                                     COMMUNICATION_MODE_KRYO,
                                     "localhost",
                                     behaviorModule.getMessager());
      behaviorUI.selectBehavior(ExploreAreaBehavior.DEFINITION);

      behaviorUI.addOnCloseRequestListener(() -> ThreadTools.startAThread(() -> {
         destroy();
         Runtime.getRuntime().exit(0);
      }, "DestroyViaUI"));

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "DestroyViaKill"));
   }

   @Override
   protected boolean destroy()
   {
      boolean destroy = super.destroy();

      if (destroy)
      {
         behaviorUI.closeMessager();
         behaviorModule.destroy();
         perceptionStack.destroy();
      }

      return destroy;
   }

   public static void main(String[] args)
   {
      AtlasSimulationBasics.runOrLogToFile(AtlasExploreAreaBehaviorDemo.class);
   }
}
