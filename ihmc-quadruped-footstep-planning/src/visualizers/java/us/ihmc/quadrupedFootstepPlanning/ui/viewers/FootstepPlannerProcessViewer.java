package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.QuadrupedPawPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.SimplePlanarRegionPawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawStepGraph;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCost;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCostBuilder;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeOccupancyMapRenderer;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeOccupancyMapSequenceRenderer;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;

import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerProcessViewer extends AnimationTimer
{
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final NodeOccupancyMapRenderer allValidNodesRenderer;
   private final NodeOccupancyMapRenderer allInvalidNodesRenderer;
   private final NodeOccupancyMapSequenceRenderer nodeExpansionPlaybackRenderer;

   private final Map<QuadrupedPawPlannerNodeRejectionReason, NodeOccupancyMapRenderer> rejectedNodesByReasonRenderers = new HashMap<>();

   private final AtomicReference<QuadrupedPawPlannerNodeRejectionReason> rejectionReasonToShow;
   private final AtomicReference<Boolean> showNodesByRejectionReason;

   private static final double cellOpacity = 0.9;
   private static final Color parentCellColor = Color.rgb(50, 205, 50, cellOpacity);
   private static final Color validCellColor = Color.rgb(0, 0, 139, cellOpacity);
   private static final Color currentValidCellColor = Color.rgb(0, 0, 255, cellOpacity);
   private static final Color rejectedCellColor = Color.rgb(139, 0, 0, cellOpacity);
   private static final Color currentRejectedCellColor = Color.rgb(255, 0, 0, cellOpacity);
   private static final Color rejectionByReasonColor = Color.rgb(20, 20, 20, cellOpacity);

   private final PawNodeCost costCalculator;
   private final PawStepGraph graph = new PawStepGraph();

   public FootstepPlannerProcessViewer(Messager messager)
   {
      PawPlannerParameters parameters = new DefaultPawPlannerParameters();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters, parameters::getProjectInsideDistance,
                                                                                      parameters::getProjectInsideUsingConvexHull,
                                                                                      true);
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();

      PawNodeCostBuilder costBuilder = new PawNodeCostBuilder();
      costBuilder.setPawPlannerParameters(parameters);
      costBuilder.setXGaitSettings(xGaitSettings);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);

      costCalculator = costBuilder.buildCost();



      allValidNodesRenderer = new NodeOccupancyMapRenderer(messager, executorService);
      allInvalidNodesRenderer = new NodeOccupancyMapRenderer(messager, executorService);

      nodeExpansionPlaybackRenderer = new NodeOccupancyMapSequenceRenderer(messager, parentCellColor, currentValidCellColor, currentRejectedCellColor, executorService);




      for (QuadrupedPawPlannerNodeRejectionReason rejectionReason : QuadrupedPawPlannerNodeRejectionReason.values)
      {
         NodeOccupancyMapRenderer renderer = new NodeOccupancyMapRenderer(messager, executorService);
         rejectedNodesByReasonRenderers.put(rejectionReason, renderer);
      }

      messager.registerTopicListener(PawPlannerMessagerAPI.ComputePathTopic, data -> reset());

      messager.registerTopicListener(PawPlannerMessagerAPI.ShowAllValidNodesTopic, allValidNodesRenderer::show);
      messager.registerTopicListener(PawPlannerMessagerAPI.ShowAllInvalidNodesTopic, allInvalidNodesRenderer::show);
      messager.registerTopicListener(PawPlannerMessagerAPI.ShowNodesThisTickTopic, nodeExpansionPlaybackRenderer::show);


      messager.registerTopicListener(PawPlannerMessagerAPI.NodesThisTickTopic, this::handleNodesThisTick);
      messager.registerTopicListener(PawPlannerMessagerAPI.NodesRejectedThisTickTopic, this::handleNodesRejectedThisTick);
      messager.registerTopicListener(PawPlannerMessagerAPI.PlannerThoughtPlaybackFractionTopic, this::handlePlaybackFraction);
      messager.registerTopicListener(PawPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings::set);

      rejectionReasonToShow = messager.createInput(PawPlannerMessagerAPI.RejectionReasonToShowTopic, QuadrupedPawPlannerNodeRejectionReason.OBSTACLE_BLOCKING_STEP);
      showNodesByRejectionReason = messager.createInput(PawPlannerMessagerAPI.ShowNodesRejectedByReasonTopic, false);

      root.getChildren().addAll(allValidNodesRenderer.getRoot(), allInvalidNodesRenderer.getRoot(), nodeExpansionPlaybackRenderer.getRoot());
      for (QuadrupedPawPlannerNodeRejectionReason rejectionReason : QuadrupedPawPlannerNodeRejectionReason.values)
         root.getChildren().add(rejectedNodesByReasonRenderers.get(rejectionReason).getRoot());
   }

   public void start()
   {
      super.start();

      allValidNodesRenderer.start();
      allInvalidNodesRenderer.start();
      nodeExpansionPlaybackRenderer.start();
      for (QuadrupedPawPlannerNodeRejectionReason rejectionReason : QuadrupedPawPlannerNodeRejectionReason.values)
         rejectedNodesByReasonRenderers.get(rejectionReason).start();
   }

   public void stop()
   {
      super.stop();

      allValidNodesRenderer.stop();
      allInvalidNodesRenderer.stop();
      nodeExpansionPlaybackRenderer.stop();
      for (QuadrupedPawPlannerNodeRejectionReason rejectionReason : QuadrupedPawPlannerNodeRejectionReason.values)
         rejectedNodesByReasonRenderers.get(rejectionReason).stop();

      executorService.shutdownNow();
   }

   private void reset()
   {
      allValidNodesRenderer.reset();
      allInvalidNodesRenderer.reset();
      nodeExpansionPlaybackRenderer.reset();
      for (QuadrupedPawPlannerNodeRejectionReason rejectionReason : QuadrupedPawPlannerNodeRejectionReason.values)
         rejectedNodesByReasonRenderers.get(rejectionReason).reset();
   }

   private synchronized void handlePlaybackFraction(Number playbackFraction)
   {
      double alpha = playbackFraction.doubleValue();
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      nodeExpansionPlaybackRenderer.requestSpecificPercentageInPlayback(alpha);
   }

   private synchronized void handleNodesThisTick(HashMap<PawNode, Pair<List<PawNode>, List<PawNode>>> validNodesThisTick)
   {
      Set<PawNode> validNodesToRender = new HashSet<>();
      Set<PawNode> invalidNodesToRender = new HashSet<>();
      Set<PawNode> parentNodes = new HashSet<>();
      Set<List<PawNode>> optimalPaths = new HashSet<>();
      for (PawNode parentNode : validNodesThisTick.keySet())
      {
         if (parentNode == null)
            continue;

         List<PawNode> validNodes = validNodesThisTick.get(parentNode).getLeft();
         List<PawNode> invalidNodes = validNodesThisTick.get(parentNode).getRight();

         for (PawNode node : validNodes)
         {
            if (node != null)
            {
               validNodesToRender.add(node);
               if (!graph.doesNodeExist(parentNode))
                  graph.initialize(parentNode);
               graph.checkAndSetEdge(parentNode, node, costCalculator.compute(parentNode, node));
            }
         }
         for (PawNode node : invalidNodes)
         {
            if (node != null)
               invalidNodesToRender.add(node);
         }
         parentNodes.add(parentNode);

         optimalPaths.add(graph.getPathFromStart(parentNode));
      }

      if (validNodesToRender.size() > 0)
         allValidNodesRenderer.processNodesToRenderOnThread(validNodesToRender, validCellColor);
      if (invalidNodesToRender.size() > 0)
         allInvalidNodesRenderer.processNodesToRenderOnThread(invalidNodesToRender, rejectedCellColor);
      if (parentNodes.size() > 0 && validNodesToRender.size() > 0 && optimalPaths.size() > 0)
         nodeExpansionPlaybackRenderer.processNodesToRender(parentNodes, validNodesToRender, invalidNodesToRender, optimalPaths);
   }

   private synchronized void handleNodesRejectedThisTick(HashMap<QuadrupedPawPlannerNodeRejectionReason, List<PawNode>> rejectedNodes)
   {
      for (QuadrupedPawPlannerNodeRejectionReason rejectionReason : rejectedNodes.keySet())
      {
         Set<PawNode> invalidNodes = new HashSet<>(rejectedNodes.get(rejectionReason));
         if (invalidNodes.size() > 0)
            rejectedNodesByReasonRenderers.get(rejectionReason).processNodesToRenderOnThread(invalidNodes, rejectionByReasonColor);
      }
   }


   @Override
   public void handle(long now)
   {
      for (QuadrupedPawPlannerNodeRejectionReason rejectionReason : QuadrupedPawPlannerNodeRejectionReason.values)
      {
         if (rejectionReason == rejectionReasonToShow.get())
            rejectedNodesByReasonRenderers.get(rejectionReason).show(showNodesByRejectionReason.get());
         else
            rejectedNodesByReasonRenderers.get(rejectionReason).show(false);
      }
   }

   public Node getRoot()
   {
      return root;
   }
}
