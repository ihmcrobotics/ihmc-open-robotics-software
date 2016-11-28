package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPlanarRegionsList;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer implements BipedalFootstepPlannerListener
{
   private static final double FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT = 0.01;
   private SimulationConstructionSet scs;
   private YoVariableServer yoVariableServer;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final DoubleYoVariable plannerTime = new DoubleYoVariable("plannerTime", registry);
   private final IntegerYoVariable plannerUpdateIndex = new IntegerYoVariable("plannerUpdateIndex", registry);
   private final IntegerYoVariable planarRegionUpdateIndex = new IntegerYoVariable("planarRegionUpdateIndex", registry);

   private final PlanarRegionBipedalFootstepPlannerVisualizer footstepPlannerVisualizer;
   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private boolean cropBufferWhenSolutionIsFound = true;

   public static YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer createWithYoVariableServer(FullRobotModel fullRobotModel,
                                                                                                         LogModelProvider logModelProvider,
                                                                                                         SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer visualizer = new YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame);

      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("PlannerScheduler");
      YoVariableServer yoVariableServer = new YoVariableServer(visualizer.getClass(), scheduler, logModelProvider, LogSettings.FOOTSTEP_PLANNER,
                                                               FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT);
      visualizer.setYoVariableServer(yoVariableServer);

      yoVariableServer.setMainRegistry(visualizer.getYoVariableRegistry(), fullRobotModel, visualizer.getYoGraphicsListRegistry());
      yoVariableServer.start();

      return visualizer;
   }

   public static YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer createWithSimulationConstructionSet(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer visualizer = new YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame);

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
      visualizer.setSimulationConstructionSet(scs);

      scs.changeBufferSize(32000);

      scs.addYoVariableRegistry(visualizer.getYoVariableRegistry());
      scs.addYoGraphicsListRegistry(visualizer.getYoGraphicsListRegistry());

      scs.setDT(FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT, 1);

      scs.setCameraFix(-6.0, 0.0, 0.0);
      scs.setCameraPosition(-11.0, 0.0, 8.0);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      return visualizer;
   }

   public YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer(SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      int vertexBufferSize = 100;
      int meshBufferSize = 100;
      yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("planarRegionsList", vertexBufferSize, meshBufferSize, registry);
      graphicsListRegistry.registerYoGraphic("PlanarRegionsList", yoGraphicPlanarRegionsList);

      footstepPlannerVisualizer = new PlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame, registry, graphicsListRegistry);
   }

   public void setYoVariableServer(YoVariableServer yoVariableServer)
   {
      this.yoVariableServer = yoVariableServer;
   }

   public void setSimulationConstructionSet(SimulationConstructionSet scs)
   {
      this.scs = scs;
   }

   public void setCropBufferWhenSolutionIsFound(boolean cropBufferWhenSolutionIsFound)
   {
      this.cropBufferWhenSolutionIsFound = cropBufferWhenSolutionIsFound;
   }

   @Override
   public void goalWasSet(RigidBodyTransform goalLeftFootPose, RigidBodyTransform goalRightFootPose)
   {
      footstepPlannerVisualizer.goalWasSet(goalLeftFootPose, goalRightFootPose);
      plannerUpdateIndex.increment();
      tickAndUpdateSCSOrYoVariableServer();
   }

   @Override
   public void nodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand)
   {
      footstepPlannerVisualizer.nodeSelectedForExpansion(nodeToExpand);
      plannerUpdateIndex.increment();
      tickAndUpdateSCSOrYoVariableServer();
   }

   @Override
   public void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode acceptedNode)
   {
      footstepPlannerVisualizer.nodeForExpansionWasAccepted(acceptedNode);
      plannerUpdateIndex.increment();
      tickAndUpdateSCSOrYoVariableServer();
   }

   @Override
   public void nodeForExpansionWasRejected(BipedalFootstepPlannerNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      footstepPlannerVisualizer.nodeForExpansionWasRejected(rejectedNode, reason);
      plannerUpdateIndex.increment();
      tickAndUpdateSCSOrYoVariableServer();
   }

   @Override
   public void notifyListenerSolutionWasFound()
   {
      if ((scs != null) && (cropBufferWhenSolutionIsFound))
      {
         scs.cropBuffer();
      }

      plannerUpdateIndex.set(0);
      tickAndUpdateSCSOrYoVariableServer();
   }

   @Override
   public void notifyListenerSolutionWasNotFound()
   {
      if ((scs != null) && (cropBufferWhenSolutionIsFound))
      {
         scs.cropBuffer();
      }

      plannerUpdateIndex.set(0);
      tickAndUpdateSCSOrYoVariableServer();
   }

   //   private Graphics3DNode node;

   @Override
   public void planarRegionsListSet(PlanarRegionsList planarRegionsList)
   {
      //      if (node != null)
      //      {
      //         scs.removeGraphics3dNode(node);
      //      }
      //      if (planarRegionsList == null)
      //         return;
      //
      //      planarRegionsListIndex.increment();
      //      Graphics3DObject graphics3dObject = new Graphics3DObject();
      //      graphics3dObject.addPlanarRegionsList(planarRegionsList, YoAppearance.Blue(), YoAppearance.Purple(), YoAppearance.Pink(), YoAppearance.Orange(), YoAppearance.Brown());
      //      node = scs.addStaticLinkGraphics(graphics3dObject);
      //

      planarRegionUpdateIndex.set(0);

      yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList);
      while (!yoGraphicPlanarRegionsList.isQueueEmpty())
      {
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
         planarRegionUpdateIndex.increment();
         tickAndUpdateSCSOrYoVariableServer();
      }

      tickAndUpdateSCSOrYoVariableServer();
   }

   private void tickAndUpdateSCSOrYoVariableServer()
   {
      plannerTime.add(FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT);

      if (yoVariableServer != null)
      {
         yoVariableServer.update(TimeTools.secondsToNanoSeconds(plannerTime.getDoubleValue()));
         //         ThreadTools.sleep(2L);
      }

      if (scs != null)
      {
         scs.setTime(scs.getTime() + FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT);
         scs.tickAndUpdate();
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return graphicsListRegistry;
   }
}
