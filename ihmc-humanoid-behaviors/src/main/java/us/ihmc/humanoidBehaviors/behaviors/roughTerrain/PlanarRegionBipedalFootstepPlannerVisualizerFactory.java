package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PlanarRegionBipedalFootstepPlannerVisualizerFactory
{
   public static PlanarRegionBipedalFootstepPlannerVisualizer createWithYoVariableServer(double dtForViz, FullRobotModel fullRobotModel,
                                                                                         LogModelProvider logModelProvider,
                                                                                         SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, String namePrefix)
   {
      YoVariableRegistry registry = new YoVariableRegistry(PlanarRegionBipedalFootstepPlannerVisualizerFactory.class.getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionBipedalFootstepPlannerVisualizer footstepPlannerVisualizer = new PlanarRegionBipedalFootstepPlannerVisualizer(10, footPolygonsInSoleFrame,
                                                                                                                                registry, graphicsListRegistry);

      PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();
      YoVariableServer yoVariableServer = new YoVariableServer(namePrefix + PlanarRegionBipedalFootstepPlannerVisualizerFactory.class.getSimpleName(), scheduler, logModelProvider,
                                                               LogSettings.FOOTSTEP_PLANNER, dtForViz);
      yoVariableServer.setSendKeepAlive(true);
      footstepPlannerVisualizer.setTickAndUpdatable(yoVariableServer);

      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), graphicsListRegistry);
      try
      {
         yoVariableServer.start();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      return footstepPlannerVisualizer;
   }

   public static PlanarRegionBipedalFootstepPlannerVisualizer createWithSimulationConstructionSet(double dtForViz,
                                                                                                  SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      YoVariableRegistry registry = new YoVariableRegistry(PlanarRegionBipedalFootstepPlannerVisualizerFactory.class.getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionBipedalFootstepPlannerVisualizer footstepPlannerVisualizer = new PlanarRegionBipedalFootstepPlannerVisualizer(10, footPolygonsInSoleFrame,
                                                                                                                                registry, graphicsListRegistry);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));

      footstepPlannerVisualizer.setTickAndUpdatable(scs);

      scs.changeBufferSize(32000);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      scs.setDT(dtForViz, 1);

      scs.setGroundVisible(false);
      scs.startOnAThread();

      return footstepPlannerVisualizer;
   }

}
