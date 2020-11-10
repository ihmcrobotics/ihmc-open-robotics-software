package us.ihmc.atlas;

import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.atlas.behaviors.tools.AtlasSimulationBasics;
import us.ihmc.avatar.environments.PhaseOneDemoEnvironment;
import us.ihmc.avatar.environments.PhaseOneDemoEnvironment.StartingLocation;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehavior;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.humanoidBehaviors.tools.perception.MultisenseHeadStereoSimulator;
import us.ihmc.humanoidBehaviors.tools.perception.PeriodicPlanarRegionPublisher;
import us.ihmc.humanoidBehaviors.tools.perception.RealsensePelvisSimulator;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.simulation.EnvironmentInitialSetup;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class AtlasBuildingExplorationDemo extends AtlasSimulationBasics
{
   private static boolean START_LOOK_AND_STEP_UI = Boolean.parseBoolean(System.getProperty("start.look.and.step.ui"));
   private static StartingLocation STARTING_LOCATION = StartingLocation.values()[Integer.parseInt(System.getProperty("starting.location", "0"))];
   private static boolean CREATE_PUSH_DOOR = Boolean.parseBoolean(System.getProperty("create.push.door", "true"));
   private static boolean CREATE_PULL_DOOR = Boolean.parseBoolean(System.getProperty("create.pull.door", "true"));
   private static boolean CREATE_DEBRIS = Boolean.parseBoolean(System.getProperty("create.debris", "false"));
   private static boolean CREATE_BARREL = Boolean.parseBoolean(System.getProperty("create.barrel", "false"));
   private static boolean CREATE_STAIRS = Boolean.parseBoolean(System.getProperty("create.stairs", "true"));
   private static boolean CREATE_CINDER_BLOCK_FIELD = Boolean.parseBoolean(System.getProperty("create.cinder.block.field", "true"));

   private final AtomicBoolean ignoreDebris = new AtomicBoolean(false);
   private final PhaseOneDemoEnvironment environment;
   private boolean lastIgnoreDebrisValueUsedByRealsense = true;

   {
      environment = new PhaseOneDemoEnvironment(CREATE_PUSH_DOOR, CREATE_PULL_DOOR, CREATE_DEBRIS, CREATE_BARREL, CREATE_STAIRS, CREATE_CINDER_BLOCK_FIELD);
      environmentInitialSetups.add(new EnvironmentInitialSetup(environment::getDebrisRegions,
                                                               environment,
                                                               STARTING_LOCATION.getPose().getZ(),
                                                               STARTING_LOCATION.getPose().getYaw(),
                                                               STARTING_LOCATION.getPose().getX(),
                                                               STARTING_LOCATION.getPose().getY()));
   }

   public AtlasBuildingExplorationDemo()
   {
      if (CREATE_STAIRS)
      {
         CREATE_MORE_FOOT_CONTACT_POINTS = true;
         numberOfContactPointsX = 4;
         numberOfContactPointsY = 3;
      }
      if (CREATE_PUSH_DOOR || CREATE_PULL_DOOR || CREATE_BARREL)
      {
         CREATE_HAND_CONTACT_POINTS = true;
      }

      AtlasRobotModel robotModel = createRobotModel();

      JToggleButton ignoreDebrisButton = new JToggleButton("Ignore debris");
      ignoreDebrisButton.addChangeListener(e -> ignoreDebris.set(ignoreDebrisButton.isSelected()));

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setStartingLocationOffset(STARTING_LOCATION.getPose().getPosition(), STARTING_LOCATION.getPose().getYaw());
      simulationStarter.getSCSInitialSetup().setTimePerRecordTick(50 * robotModel.getControllerDT());
      simulationStarter.getSCSInitialSetup().setSimulationDataBufferSize(10);
      List<DRCSimulationTools.Modules> modulesToStart = new ArrayList<>();
      modulesToStart.add(DRCSimulationTools.Modules.SIMULATION);
      modulesToStart.add(DRCSimulationTools.Modules.NETWORK_PROCESSOR);
      modulesToStart.add(DRCSimulationTools.Modules.SENSOR_MODULE);
      modulesToStart.add(DRCSimulationTools.Modules.FIDUCIAL_DETECTOR);
      modulesToStart.add(DRCSimulationTools.Modules.OBJECT_DETECTOR);
      modulesToStart.add(DRCSimulationTools.Modules.BEHAVIOR_MODULE);
      modulesToStart.add(DRCSimulationTools.Modules.FOOTSTEP_PLANNING_TOOLBOX);
      LogTools.info("Starting simulation modules");

      String[] args = {"-m " + robotModel.getAtlasVersion().name()};
      DRCSimulationTools.startSimulation(simulationStarter, null, args, null, modulesToStart);

      simulationStarter.getSimulationConstructionSet().addButton(ignoreDebrisButton);

      // Start Look and Step behavior
      BehaviorRegistry behaviorRegistry = BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION, TraverseStairsBehavior.DEFINITION);
      BehaviorModule.createInterprocess(behaviorRegistry, robotModel);

      if (START_LOOK_AND_STEP_UI)
      {
         BehaviorUI.createInterprocess(BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION), robotModel, "127.0.0.1");
      }

      ThreadTools.startAsDaemon(this::startPerceptionStack, "PerceptionStack");
      wakeUpToolboxes(robotModel);
   }

   private void wakeUpToolboxes(AtlasRobotModel robotModel)
   {
      // Start object detector toolbox
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "toolboxes");
      String robotName = robotModel.getSimpleRobotName();
      IHMCROS2Publisher<ToolboxStateMessage> fiducialDetectorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                            ToolboxStateMessage.class,
                                                                                                            FiducialDetectorToolboxModule.getInputTopic(
                                                                                                                  robotName));
      IHMCROS2Publisher<ToolboxStateMessage> objectDetectorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                          ToolboxStateMessage.class,
                                                                                                          ObjectDetectorToolboxModule.getInputTopic(robotName));

      new PausablePeriodicThread("ToolboxWaker", 1.0, () ->
      {
         ToolboxStateMessage wakeUpMessage = new ToolboxStateMessage();
         wakeUpMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
         fiducialDetectorPublisher.publish(wakeUpMessage);
         objectDetectorPublisher.publish(wakeUpMessage);
      }).start();
   }

   private void startPerceptionStack()
   {
      // Publish planar regions
      PlanarRegionsList environmentWithoutDebrisRegions = environment.getEnvironmentRegions();
      PlanarRegionsList debrisRegions = environment.getDebrisRegions();
      PlanarRegionsList environmentWithDebrisRegions = new PlanarRegionsList();

      for (int i = 0; i < environmentWithoutDebrisRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = environmentWithoutDebrisRegions.getPlanarRegion(i);
         region.setRegionId(environmentWithDebrisRegions.getNumberOfPlanarRegions());
         environmentWithDebrisRegions.addPlanarRegion(region);
      }
      for (int i = 0; i < debrisRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = debrisRegions.getPlanarRegion(i);
         region.setRegionId(environmentWithDebrisRegions.getNumberOfPlanarRegions());
         environmentWithDebrisRegions.addPlanarRegion(region);
      }

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, ROS2Tools.REA_NODE_NAME);

      PlanarRegionSLAMMapper realsensePlanarRegionSLAM = new PlanarRegionSLAMMapper();

      AtlasRobotModel robotModel = createRobotModel();
      MultisenseHeadStereoSimulator multisense = new MultisenseHeadStereoSimulator(environmentWithDebrisRegions, robotModel, ros2Node);
      RealsensePelvisSimulator realsense = new RealsensePelvisSimulator(environmentWithDebrisRegions, robotModel, ros2Node);

      // might be a weird delay with threads at 0.5 hz depending on each other
      double period = 3.0;
      new PeriodicPlanarRegionPublisher(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, period, () ->
      {
         multisense.setMap(ignoreDebris.get() ? environmentWithoutDebrisRegions : environmentWithDebrisRegions);
         return multisense.computeRegions();
      }).start();
      new PeriodicPlanarRegionPublisher(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, period, () ->
      {
         boolean ignoreDebrisLocal = ignoreDebris.get();
         if (ignoreDebrisLocal != lastIgnoreDebrisValueUsedByRealsense)
         {
            realsensePlanarRegionSLAM.clear();
            lastIgnoreDebrisValueUsedByRealsense = ignoreDebrisLocal;
         }
         realsense.setMap(ignoreDebrisLocal ? environmentWithoutDebrisRegions : environmentWithDebrisRegions);
         return realsensePlanarRegionSLAM.update(realsense.computeRegions());
      }).start();
   }

   public static void main(final String[] args)
   {
      new AtlasBuildingExplorationDemo();
   }
}
