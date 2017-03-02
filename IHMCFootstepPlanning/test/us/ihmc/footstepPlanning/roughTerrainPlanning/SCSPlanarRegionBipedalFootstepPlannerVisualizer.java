package us.ihmc.footstepPlanning.roughTerrainPlanning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SCSPlanarRegionBipedalFootstepPlannerVisualizer
{
   public static PlanarRegionBipedalFootstepPlannerVisualizer createWithSimulationConstructionSet(double dtForViz, SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame, YoVariableRegistry registryToAdd)
   {
      return createWithSimulationConstructionSet(dtForViz, new Point3D(0.0, 0.0, 0.0), new Point3D(-5.0, 0.0, 5.0), footPolygonsInSoleFrame, registryToAdd);
   }

   public static PlanarRegionBipedalFootstepPlannerVisualizer createWithSimulationConstructionSet(double dtForViz, Point3D cameraFix, Point3D cameraPosition, 
                                                                                                  SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame, 
                                                                                                  YoVariableRegistry registryToAdd)
   {
      YoVariableRegistry registry = new YoVariableRegistry(SCSPlanarRegionBipedalFootstepPlannerVisualizer.class.getSimpleName());
      registry.addChild(registryToAdd);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionBipedalFootstepPlannerVisualizer footstepPlannerVisualizer = new PlanarRegionBipedalFootstepPlannerVisualizer(10, footPolygonsInSoleFrame, registry, graphicsListRegistry);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
      
      footstepPlannerVisualizer.setTickAndUpdatable(scs);

      scs.changeBufferSize(32000);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      scs.setDT(dtForViz, 1);

      scs.setCameraFix(cameraFix);
      scs.setCameraPosition(cameraPosition);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      return footstepPlannerVisualizer;
   }
}