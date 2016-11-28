package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.graphics3DDescription.structure.Graphics3DNode;
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
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer implements BipedalFootstepPlannerListener
{
   private static final double FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT = 0.001;
   private SimulationConstructionSet scs;
   private YoVariableServer yoVariableServer;

   private final DoubleYoVariable plannerTime;
   private final IntegerYoVariable plannerUpdateIndex;
   private final IntegerYoVariable planarRegionUpdateIndex;

   private final PlanarRegionBipedalFootstepPlannerVisualizer footstepPlannerVisualizer;
   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private boolean cropBufferWhenSolutionIsFound = true;

   private final IntegerYoVariable planarRegionsListIndex;
   private final YoVariableRegistry registry;


   public static YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer createWithYoVariableServer(FullRobotModel fullRobotModel, LogModelProvider logModelProvider, SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer visualizer = new YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer(fullRobotModel, logModelProvider, footPolygonsInSoleFrame);
      return visualizer;
   }

   public YoVariableServerPlanarRegionBipedalFootstepPlannerVisualizer(FullRobotModel fullRobotModel, LogModelProvider logModelProvider, SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame)
   {
      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("PlannerScheduler");
      yoVariableServer = new YoVariableServer(getClass(), scheduler, logModelProvider, LogSettings.FOOTSTEP_PLANNER, FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT);

     registry = new YoVariableRegistry(getClass().getSimpleName());
     YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

     plannerTime = new DoubleYoVariable("plannerTime", registry);
     plannerUpdateIndex = new IntegerYoVariable("plannerUpdateIndex", registry);
     planarRegionUpdateIndex = new IntegerYoVariable("planarRegionUpdateIndex", registry);


//      scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.changeBufferSize(32000);

//      registry = scs.getRootRegistry();
//      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

     int vertexBufferSize = 100;
     int meshBufferSize = 100;
     yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("planarRegionsList", vertexBufferSize, meshBufferSize, registry);
     graphicsListRegistry.registerYoGraphic("PlanarRegionsList", yoGraphicPlanarRegionsList);

     footstepPlannerVisualizer = new PlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame, registry, graphicsListRegistry);

     planarRegionsListIndex = new IntegerYoVariable("planarRegionsListIndex", footstepPlannerVisualizer.getYoVariableRegistry());

//      scs.addYoGraphicsListRegistry(graphicsListRegistry);
//      scs.setDT(0.1, 1);
//
//      scs.setCameraFix(-6.0, 0.0, 0.0);
//      scs.setCameraPosition(-11.0, 0.0, 8.0);
//      scs.setGroundVisible(false);
//      scs.startOnAThread();


      yoVariableServer.setMainRegistry(registry, fullRobotModel, graphicsListRegistry);
      yoVariableServer.start();
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
      tickAndUpdateSCS();
   }

   @Override
   public void nodeSelectedForExpansion(BipedalFootstepPlannerNode nodeToExpand)
   {
      footstepPlannerVisualizer.nodeSelectedForExpansion(nodeToExpand);
      plannerUpdateIndex.increment();
      tickAndUpdateSCS();
   }

   @Override
   public void nodeForExpansionWasAccepted(BipedalFootstepPlannerNode acceptedNode)
   {
      footstepPlannerVisualizer.nodeForExpansionWasAccepted(acceptedNode);
      plannerUpdateIndex.increment();
      tickAndUpdateSCS();
   }

   @Override
   public void nodeForExpansionWasRejected(BipedalFootstepPlannerNode rejectedNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      footstepPlannerVisualizer.nodeForExpansionWasRejected(rejectedNode, reason);
      plannerUpdateIndex.increment();
      tickAndUpdateSCS();
   }

   @Override
   public void notifyListenerSolutionWasFound()
   {
//      if (cropBufferWhenSolutionIsFound)
//      {
//         scs.cropBuffer();
//      }
//      scs.setTime(0.0);

      plannerUpdateIndex.set(0);
      tickAndUpdateSCS();
   }

   @Override
   public void notifyListenerSolutionWasNotFound()
   {
//      if (cropBufferWhenSolutionIsFound)
//      {
//         scs.cropBuffer();
//      }
//      scs.setTime(0.0);

      plannerUpdateIndex.set(0);
      tickAndUpdateSCS();
   }

   private Graphics3DNode node;

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
      while(!yoGraphicPlanarRegionsList.isQueueEmpty())
      {
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
         planarRegionUpdateIndex.increment();
         tickAndUpdateSCS();
      }

      tickAndUpdateSCS();
   }

   private void tickAndUpdateSCS()
   {
      plannerTime.add(FOOTSTEP_PLANNER_YO_VARIABLE_SERVER_DT);
      yoVariableServer.update(TimeTools.secondsToNanoSeconds(plannerTime.getDoubleValue()));
      ThreadTools.sleep(2L);
//      scs.setTime(scs.getTime() + 1.0);
//      scs.tickAndUpdate();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
//      return scs.getRootRegistry();
      return registry;
   }
}
