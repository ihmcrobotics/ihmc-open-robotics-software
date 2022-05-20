package us.ihmc.atlas;

import java.util.List;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.RoughTerrainEnvironmentStartingLocation;
import us.ihmc.avatar.YoGraphicPlanarRegionsController;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.simulationConstructionSetTools.util.environments.RoughTerrainEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class AtlasGDXRoughTerrainSimulation
{
   // Increase to 10 when you want the sims to run a little faster and don't need the data.
   private final int recordFrequencySpeedup = 1;
   
   // Set to true if you are walking over steps or things where your foot might overhang a little.
   private boolean addExtraContactPoints = false;
   
   public AtlasGDXRoughTerrainSimulation()
   {
      FootContactPoints<RobotSide> simulationContactPoints = null;
      if (addExtraContactPoints)
      {
         int nContactPointsX = 5;
         int nContactPointsY = 4;
         boolean edgePointsOnly = true;
         simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, nContactPointsX, nContactPointsY, edgePointsOnly, false);
      }

      AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
      AtlasRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel(version, simulationContactPoints);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new RoughTerrainEnvironment());

      ROS2Node planarRegionsROS2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "planar_regions_visualizer_node");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ROS2Topic<PlanarRegionsListMessage> topicName = ControllerAPIDefinition.getInputTopic(robotModel.getSimpleRobotName()).withTypeName(PlanarRegionsListMessage.class);
      YoGraphicPlanarRegionsController yoGraphicPlanarRegionsController = new YoGraphicPlanarRegionsController(planarRegionsROS2Node, yoGraphicsListRegistry, topicName);

      DRCSCSInitialSetup scsInitialSetup = simulationStarter.getSCSInitialSetup();
      int recordFrequency = recordFrequencySpeedup * scsInitialSetup.getRecordFrequency();
      scsInitialSetup.setRecordFrequency(recordFrequency);
      simulationStarter.setRunMultiThreaded(true);

      simulationStarter.setInitializeEstimatorToActual(true);

      DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter, null, null, RoughTerrainEnvironmentStartingLocation.values());
      SimulationConstructionSet scs = simulationStarter.getAvatarSimulation().getSimulationConstructionSet();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      simulationStarter.getAvatarSimulation().getHumanoidFloatingRootJointRobot().setController(yoGraphicPlanarRegionsController);
      
      boolean printOutAllYoVariables = false;
      if (printOutAllYoVariables)
         printOutAllYoVariables(simulationStarter);
   }

   private void printOutAllYoVariables(DRCSimulationStarter simulationStarter)
   {
      SimulationConstructionSet simulationConstructionSet = simulationStarter.getSimulationConstructionSet();
      YoRegistry rootRegistry = simulationConstructionSet.getRootRegistry();

      List<YoVariable> allVariablesIncludingDescendants = rootRegistry.collectSubtreeVariables();
      System.out.println("Size = " + allVariablesIncludingDescendants.size());
      for (YoVariable yoVariable : allVariablesIncludingDescendants)
      {
         System.out.println(yoVariable.getName());
      }
      System.out.println("Size = " + allVariablesIncludingDescendants.size());
   }

   public static void main(String[] args)
   {
      new AtlasGDXRoughTerrainSimulation();
   }

}
