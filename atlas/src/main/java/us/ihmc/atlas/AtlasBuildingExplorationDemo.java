package us.ihmc.atlas;

import us.ihmc.atlas.behaviors.coordinator.AtlasBuildingExplorationBehaviorUI;
import us.ihmc.atlas.behaviors.tools.AtlasSimulationBasics;
import us.ihmc.avatar.environments.PhaseOneDemoEnvironment;
import us.ihmc.avatar.environments.PhaseOneDemoEnvironment.StartingLocation;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionSLAMMapper;
import us.ihmc.humanoidBehaviors.tools.perception.MultisenseHeadStereoSimulator;
import us.ihmc.humanoidBehaviors.tools.perception.PeriodicPlanarRegionPublisher;
import us.ihmc.humanoidBehaviors.tools.perception.RealsensePelvisSimulator;
import us.ihmc.humanoidBehaviors.ui.simulation.EnvironmentInitialSetup;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class AtlasBuildingExplorationDemo extends AtlasSimulationBasics
{
   private static boolean USE_KINEMATICS_SIMULATION = Boolean.parseBoolean(System.getProperty("use.kinematics.simulation", "false"));
   private static StartingLocation STARTING_LOCATION = StartingLocation.values()[Integer.parseInt(System.getProperty("starting.location", "1"))];
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
      selectEnvironment();
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

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setPubSubImplementation(COMMUNICATION_MODE_ROS2.getPubSubImplementation());
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setLogToFile(LOG_TO_FILE);
      simulationStarter.setStartingLocationOffset(STARTING_LOCATION.getPose().getPosition(), STARTING_LOCATION.getPose().getYaw());
      simulationStarter.getSCSInitialSetup().setTimePerRecordTick(50 * robotModel.getControllerDT());
      simulationStarter.getSCSInitialSetup().setSimulationDataBufferSize(10);
      List<DRCSimulationTools.Modules> modulesToStart = new ArrayList<>();
      if (!USE_KINEMATICS_SIMULATION)
      {
         modulesToStart.add(DRCSimulationTools.Modules.SIMULATION);
         modulesToStart.add(DRCSimulationTools.Modules.SENSOR_MODULE);
      }
      modulesToStart.add(DRCSimulationTools.Modules.NETWORK_PROCESSOR);
      modulesToStart.add(DRCSimulationTools.Modules.FIDUCIAL_DETECTOR);
      modulesToStart.add(DRCSimulationTools.Modules.OBJECT_DETECTOR);
//      modulesToStart.add(DRCSimulationTools.Modules.BEHAVIOR_MODULE);
      modulesToStart.add(DRCSimulationTools.Modules.FOOTSTEP_PLANNING_TOOLBOX);
      LogTools.info("Starting simulation modules");

      String[] args = {"-m " + robotModel.getAtlasVersion().name()};
      DRCSimulationTools.startSimulation(simulationStarter, null, args, null, modulesToStart);

      if (USE_KINEMATICS_SIMULATION)
      {
         ThreadTools.startAsDaemon(this::lidarAndCameraSimulator, "LidarAndCamera");
         kinematicSimulation();
      }
      else
      {
         JToggleButton ignoreDebrisButton = new JToggleButton("Ignore debris");
         ignoreDebrisButton.addChangeListener(e -> ignoreDebris.set(ignoreDebrisButton.isSelected()));
         simulationStarter.getSimulationConstructionSet().addButton(ignoreDebrisButton);
         simulationStarter.getSimulationConstructionSet().skipLoadingDefaultConfiguration();
         simulationStarter.getSimulationConstructionSet().setupGraph("t");
      }

      LogTools.info("Starting building exploration behavior");
      AtlasBuildingExplorationBehaviorUI.start(createRobotModel(), COMMUNICATION_MODE_ROS2, COMMUNICATION_MODE_KRYO);

      ThreadTools.startAsDaemon(this::startPerceptionStack, "PerceptionStack");
   }

   private void startPerceptionStack()
   {
      // Publish planar regions
      PlanarRegionsList environmentWithoutDebrisRegions = environment.getEnvironmentRegions();
      PlanarRegionsList environmentWithDebrisRegions = environment.getEnvironmentWithDebrisRegions();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(COMMUNICATION_MODE_ROS2.getPubSubImplementation(), ROS2Tools.REA_NODE_NAME);

      PlanarRegionSLAMMapper realsensePlanarRegionSLAM = new PlanarRegionSLAMMapper();

      AtlasRobotModel robotModel = createRobotModel();
      double range = 20.0;
      int sphereScanSize = 50000;
      MultisenseHeadStereoSimulator multisense = new MultisenseHeadStereoSimulator(environmentWithDebrisRegions, robotModel, ros2Node, range, sphereScanSize);
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

   @Override
   protected boolean destroy()
   {
      boolean destroy = super.destroy();

      if (destroy)
      {
         // TODO
         // destroy stuff
      }

      return destroy;
   }

   public static void main(final String[] args)
   {
      AtlasSimulationBasics.runOrLogToFile(AtlasBuildingExplorationDemo.class);
   }
}
