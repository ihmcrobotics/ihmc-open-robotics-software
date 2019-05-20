package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.quadrupedFootstepPlanning.ui.SimpleFootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeOccupancyMapRenderer;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeOccupancyMapSequenceRenderer;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;

import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class FootstepPlannerProcessViewer extends AnimationTimer
{
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();

   private final NodeOccupancyMapRenderer allValidNodesRenderer;
   private final NodeOccupancyMapRenderer allInvalidNodesRenderer;
   private final NodeOccupancyMapSequenceRenderer nodeExpansionPlaybackRenderer;

   private final Map<QuadrupedFootstepPlannerNodeRejectionReason, NodeOccupancyMapRenderer> rejectedNodesByReasonRenderers = new HashMap<>();

   private final AtomicReference<QuadrupedFootstepPlannerNodeRejectionReason> rejectionReasonToShow;
   private final AtomicReference<Boolean> showNodesByRejectionReason;

   private static final double cellOpacity = 0.9;
   private static final Color parentCellColor = Color.rgb(50, 205, 50, cellOpacity);
   private static final Color validCellColor = Color.rgb(0, 0, 139, cellOpacity);
   private static final Color currentValidCellColor = Color.rgb(0, 0, 255, cellOpacity);
   private static final Color rejectedCellColor = Color.rgb(139, 0, 0, cellOpacity);
   private static final Color currentRejectedCellColor = Color.rgb(255, 0, 0, cellOpacity);
   private static final Color rejectionByReasonColor = Color.rgb(20, 20, 20, cellOpacity);

   private final FootstepCost costCalculator;
   private final FootstepGraph graph = new FootstepGraph();

   public FootstepPlannerProcessViewer(Messager messager)
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                                                parameters::getProjectInsideUsingConvexHullDuringExpansion,
                                                                                                true);
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setXGaitSettings(xGaitSettings);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);

      costCalculator = costBuilder.buildCost();



      allValidNodesRenderer = new NodeOccupancyMapRenderer(messager, executorService);
      allInvalidNodesRenderer = new NodeOccupancyMapRenderer(messager, executorService);

      nodeExpansionPlaybackRenderer = new NodeOccupancyMapSequenceRenderer(messager, parentCellColor, currentValidCellColor, currentRejectedCellColor, executorService);




      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : QuadrupedFootstepPlannerNodeRejectionReason.values)
      {
         NodeOccupancyMapRenderer renderer = new NodeOccupancyMapRenderer(messager, executorService);
         rejectedNodesByReasonRenderers.put(rejectionReason, renderer);
      }

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset());

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowAllValidNodesTopic, allValidNodesRenderer::show);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowAllInvalidNodesTopic, allInvalidNodesRenderer::show);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowNodesThisTickTopic, nodeExpansionPlaybackRenderer::show);


      messager.registerTopicListener(FootstepPlannerMessagerAPI.NodesThisTickTopic, this::handleNodesThisTick);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.NodesRejectedThisTickTopic, this::handleNodesRejectedThisTick);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerThoughtPlaybackFractionTopic, this::handlePlaybackFraction);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.XGaitSettingsTopic, xGaitSettings::set);

      rejectionReasonToShow = messager.createInput(FootstepPlannerMessagerAPI.RejectionReasonToShowTopic, QuadrupedFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_STEP);
      showNodesByRejectionReason = messager.createInput(FootstepPlannerMessagerAPI.ShowNodesRejectedByReasonTopic, false);

      root.getChildren().addAll(allValidNodesRenderer.getRoot(), allInvalidNodesRenderer.getRoot(), nodeExpansionPlaybackRenderer.getRoot());
      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : QuadrupedFootstepPlannerNodeRejectionReason.values)
         root.getChildren().add(rejectedNodesByReasonRenderers.get(rejectionReason).getRoot());
   }

   public void start()
   {
      super.start();

      allValidNodesRenderer.start();
      allInvalidNodesRenderer.start();
      nodeExpansionPlaybackRenderer.start();
      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : QuadrupedFootstepPlannerNodeRejectionReason.values)
         rejectedNodesByReasonRenderers.get(rejectionReason).start();
   }

   public void stop()
   {
      super.stop();

      allValidNodesRenderer.stop();
      allInvalidNodesRenderer.stop();
      nodeExpansionPlaybackRenderer.stop();
      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : QuadrupedFootstepPlannerNodeRejectionReason.values)
         rejectedNodesByReasonRenderers.get(rejectionReason).stop();

      executorService.shutdownNow();
   }

   private void reset()
   {
      allValidNodesRenderer.reset();
      allInvalidNodesRenderer.reset();
      nodeExpansionPlaybackRenderer.reset();
      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : QuadrupedFootstepPlannerNodeRejectionReason.values)
         rejectedNodesByReasonRenderers.get(rejectionReason).reset();
   }

   private synchronized void handlePlaybackFraction(Number playbackFraction)
   {
      double alpha = playbackFraction.doubleValue();
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      nodeExpansionPlaybackRenderer.requestSpecificPercentageInPlayback(alpha);
   }

   private synchronized void handleNodesThisTick(HashMap<FootstepNode, Pair<List<FootstepNode>, List<FootstepNode>>> validNodesThisTick)
   {
      Set<FootstepNode> validNodesToRender = new HashSet<>();
      Set<FootstepNode> invalidNodesToRender = new HashSet<>();
      Set<FootstepNode> parentNodes = new HashSet<>();
      Set<List<FootstepNode>> optimalPaths = new HashSet<>();
      for (FootstepNode parentNode : validNodesThisTick.keySet())
      {
         if (parentNode == null)
            continue;

         List<FootstepNode> validNodes = validNodesThisTick.get(parentNode).getLeft();
         List<FootstepNode> invalidNodes = validNodesThisTick.get(parentNode).getRight();

         for (FootstepNode node : validNodes)
         {
            if (node != null)
            {
               validNodesToRender.add(node);
               if (!graph.doesNodeExist(parentNode))
                  graph.initialize(parentNode);
               graph.checkAndSetEdge(parentNode, node, costCalculator.compute(parentNode, node));
            }
         }
         for (FootstepNode node : invalidNodes)
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

   private synchronized void handleNodesRejectedThisTick(HashMap<QuadrupedFootstepPlannerNodeRejectionReason, List<FootstepNode>> rejectedNodes)
   {
      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : rejectedNodes.keySet())
      {
         Set<FootstepNode> invalidNodes = new HashSet<>(rejectedNodes.get(rejectionReason));
         if (invalidNodes.size() > 0)
            rejectedNodesByReasonRenderers.get(rejectionReason).processNodesToRenderOnThread(invalidNodes, rejectionByReasonColor);
      }
   }


   @Override
   public void handle(long now)
   {
      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : QuadrupedFootstepPlannerNodeRejectionReason.values)
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
