package us.ihmc.footstepPlanning.roughTerrainPlanning;

import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

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

      scs.setCameraFix(-6.0, 0.0, 0.0);
      scs.setCameraPosition(-11.0, 0.0, 8.0);
      scs.startOnAThread();
   }

   @Override
   public void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose)
   {
      footstepPlannerVisualizer.goalWasSet(goalLeftFootPose, goalRightFootPose);
      tickAndUpdateSCS();
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
   public void nodeForExpansionWasRejected(BipedalFootstepPlannerNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      footstepPlannerVisualizer.nodeForExpansionWasRejected(rejectedNode, reason);
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
      if (planarRegionsList == null) return;

      Graphics3DObject graphics3dObject = new Graphics3DObject();
      graphics3dObject.addPlanarRegionsList(planarRegionsList, YoAppearance.Blue(), YoAppearance.Purple(), YoAppearance.Pink(), YoAppearance.Orange(), YoAppearance.Brown());
      scs.addStaticLinkGraphics(graphics3dObject);

      tickAndUpdateSCS();
   }

   private void tickAndUpdateSCS()
   {
      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }
}