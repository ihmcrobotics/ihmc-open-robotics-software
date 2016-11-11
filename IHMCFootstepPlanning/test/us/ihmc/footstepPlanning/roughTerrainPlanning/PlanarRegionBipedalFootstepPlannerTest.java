package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class PlanarRegionBipedalFootstepPlannerTest extends FootstepPlannerOnRoughTerrainTest
{
   private static final boolean visualize = true;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testOnStairCase()
   {
      super.testOnStaircase(new Vector3d(0.1, 0.1, -0.1), true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleStepOnBox()
   {
      super.testSimpleStepOnBox();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRandomEnvironment()
   {
      super.testRandomEnvironment();
   }

   @Override
   public FootstepPlanner getPlanner()
   {
      PlanarRegionBipedalFootstepPlanner planner = new PlanarRegionBipedalFootstepPlanner();

      planner.setMaximumStepReach(0.4);
      planner.setMaximumStepZ(0.2);
      planner.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      planner.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      SCSPlanarRegionBipedalFootstepPlannerVisualizer visualizer = new SCSPlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame);
      planner.setBipedalFootstepPlannerListener(visualizer);

      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }

   public class SCSPlanarRegionBipedalFootstepPlannerVisualizer implements BipedalFootstepPlannerListener
   {
      private final SimulationConstructionSet scs;
      private final PlanarRegionBipedalFootstepPlannerVisualizer footstepPlannerVisualizer;

      public SCSPlanarRegionBipedalFootstepPlannerVisualizer(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
      {
         scs = new SimulationConstructionSet(new Robot("Test"));
         YoVariableRegistry parentRegistry = scs.getRootRegistry();
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

         footstepPlannerVisualizer = new PlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame, parentRegistry, graphicsListRegistry);

         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.setDT(0.1, 1);
         scs.startOnAThread();
      }

      @Override
      public void nodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand)
      {
         footstepPlannerVisualizer.nodeSelectedForExpansion(nodeToExpand);
         tickAndUpdateSCS();
      }

      @Override
      public void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode acceptedNode)
      {
         footstepPlannerVisualizer.nodeForExpansionWasAccepted(acceptedNode);
         tickAndUpdateSCS();
      }

      @Override
      public void nodeForExpansionWasRejected(BipedalFootstepPlannerNode rejectedNode)
      {
         footstepPlannerVisualizer.nodeForExpansionWasRejected(rejectedNode);
         tickAndUpdateSCS();
      }

      @Override
      public void notifyListenerSolutionWasFound()
      {
         scs.cropBuffer();
         tickAndUpdateSCS();
      }

      @Override
      public void notifyListenerSolutionWasNotFound()
      {
         scs.cropBuffer();
         tickAndUpdateSCS();
      }

      @Override
      public void planarRegionsListSet(PlanarRegionsList planarRegionsList)
      {
         Graphics3DObject graphics3dObject = new Graphics3DObject();
         graphics3dObject.addPlanarRegionsList(planarRegionsList, YoAppearance.Blue(), YoAppearance.Purple(), YoAppearance.Pink());
         scs.addStaticLinkGraphics(graphics3dObject);

         tickAndUpdateSCS();
      }

      private void tickAndUpdateSCS()
      {
         scs.setTime(scs.getTime() + 1.0);
         scs.tickAndUpdate();
      }
   }

}
