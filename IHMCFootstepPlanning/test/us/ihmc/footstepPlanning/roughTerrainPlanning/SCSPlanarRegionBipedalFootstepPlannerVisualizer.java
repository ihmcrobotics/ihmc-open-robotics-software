package us.ihmc.footstepPlanning.roughTerrainPlanning;

import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SCSPlanarRegionBipedalFootstepPlannerVisualizer
{
   public static PlanarRegionBipedalFootstepPlannerVisualizer createWithSimulationConstructionSet(double dtForViz, SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      YoVariableRegistry registry = new YoVariableRegistry(SCSPlanarRegionBipedalFootstepPlannerVisualizer.class.getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionBipedalFootstepPlannerVisualizer footstepPlannerVisualizer = new PlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame, registry, graphicsListRegistry);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
      
      footstepPlannerVisualizer.setTickAndUpdatable(scs);

      scs.changeBufferSize(32000);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      scs.setDT(dtForViz, 1);

      scs.setCameraFix(-6.0, 0.0, 0.0);
      scs.setCameraPosition(-11.0, 0.0, 8.0);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      return footstepPlannerVisualizer;
   }
}