package us.ihmc.quadrupedFootstepPlanning.ui.viewers;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.SimpleFootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeOccupancyMapRenderer;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeOccupancyMapSequenceRenderer;

import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerProcessViewer extends AnimationTimer
{
   private final List<Node> nodesToView = new ArrayList<>();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final NodeOccupancyMapRenderer allValidNodesRenderer;
   private final NodeOccupancyMapRenderer allInvalidNodesRenderer;
   private final NodeOccupancyMapSequenceRenderer validNodeExpansionPlaybackRenderer;
   private final NodeOccupancyMapSequenceRenderer invalidNodeExpansionPlaybackRenderer;

   private final Map<QuadrupedFootstepPlannerNodeRejectionReason, NodeOccupancyMapRenderer> rejectedNodesByReasonRenderers = new HashMap<>();

   private final AtomicReference<QuadrupedFootstepPlannerNodeRejectionReason> rejectionReasonToShow = new AtomicReference<>();
   private final AtomicReference<Boolean> showNodesByRejectionReason;

   private static final double cellOpacity = 0.9;
   private static final Color parentCellColor = Color.rgb(50, 205, 50, cellOpacity);
   private static final Color validCellColor = Color.rgb(0, 0, 139, cellOpacity);
   private static final Color currentValidCellColor = Color.rgb(0, 0, 255, cellOpacity);
   private static final Color rejectedCellColor = Color.rgb(139, 0, 0, cellOpacity);
   private static final Color currentRejectedCellColor = Color.rgb(255, 0, 0, cellOpacity);
   private static final Color rejectionByReasonColor = Color.rgb(20, 20, 20, cellOpacity);

   public FootstepPlannerProcessViewer(Messager messager)
   {
      allValidNodesRenderer = new NodeOccupancyMapRenderer(messager);
      nodesToView.add(allValidNodesRenderer.getRoot());
      allInvalidNodesRenderer = new NodeOccupancyMapRenderer(messager);
      nodesToView.add(allInvalidNodesRenderer.getRoot());

      validNodeExpansionPlaybackRenderer = new NodeOccupancyMapSequenceRenderer(messager, parentCellColor, currentValidCellColor);
      nodesToView.addAll(validNodeExpansionPlaybackRenderer.getNodesToView());
      invalidNodeExpansionPlaybackRenderer = new NodeOccupancyMapSequenceRenderer(messager, parentCellColor, currentRejectedCellColor);
      nodesToView.addAll(invalidNodeExpansionPlaybackRenderer.getNodesToView());

      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : QuadrupedFootstepPlannerNodeRejectionReason.values)
      {
         NodeOccupancyMapRenderer renderer = new NodeOccupancyMapRenderer(messager);
         nodesToView.add(renderer.getRoot());
         rejectedNodesByReasonRenderers.put(rejectionReason, renderer);
      }

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePathTopic, data -> reset());

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowAllValidNodesTopic, allValidNodesRenderer::show);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowAllInvalidNodesTopic, allInvalidNodesRenderer::show);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowValidNodesThisTickTopic, validNodeExpansionPlaybackRenderer::show);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowInvalidNodesThisTickTopic, invalidNodeExpansionPlaybackRenderer::show);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ValidNodesThisTickTopic, this::handleValidNodesThisTick);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.InvalidNodesThisTickTopic, this::handleInvalidNodesThisTick);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.NodesRejectedThisTickTopic, this::handleNodesRejectedThisTick);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerPlaybackFractionTopic, this::handlePlaybackFraction);

      showNodesByRejectionReason = messager.createInput(FootstepPlannerMessagerAPI.ShowNodesRejectedByReasonTopic);
   }

   private void reset()
   {
      allValidNodesRenderer.reset();
      allInvalidNodesRenderer.reset();
      validNodeExpansionPlaybackRenderer.reset();
      invalidNodeExpansionPlaybackRenderer.reset();
   }

   private synchronized void handlePlaybackFraction(Number playbackFraction)
   {
      double alpha = playbackFraction.doubleValue();
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      validNodeExpansionPlaybackRenderer.requestSpecificPercentageInPlayback(alpha);
      invalidNodeExpansionPlaybackRenderer.requestSpecificPercentageInPlayback(alpha);
   }

   private synchronized void handleValidNodesThisTick(HashMap<FootstepNode, List<FootstepNode>> validNodesThisTick)
   {
      Set<SimpleFootstepNode> validNodes = new HashSet<>();
      Set<SimpleFootstepNode> parentNodes = new HashSet<>();
      for (FootstepNode parentNode : validNodesThisTick.keySet())
      {
         validNodesThisTick.get(parentNode).forEach(node -> validNodes.add(new SimpleFootstepNode(node)));
         parentNodes.add(new SimpleFootstepNode(parentNode));
      }

      allValidNodesRenderer.processNodesToRender(validNodes, validCellColor);
      validNodeExpansionPlaybackRenderer.processNodesToRender(parentNodes, validNodes);
   }

   private synchronized void handleInvalidNodesThisTick(HashMap<FootstepNode, List<FootstepNode>> invalidNodesThisTick)
   {
      Set<SimpleFootstepNode> invalidNodes = new HashSet<>();
      Set<SimpleFootstepNode> parentNodes = new HashSet<>();
      for (FootstepNode parentNode : invalidNodesThisTick.keySet())
      {
         invalidNodesThisTick.get(parentNode).forEach(node -> invalidNodes.add(new SimpleFootstepNode(node)));
         parentNodes.add(new SimpleFootstepNode(parentNode));
      }

      allInvalidNodesRenderer.processNodesToRender(invalidNodes, rejectedCellColor);
      invalidNodeExpansionPlaybackRenderer.processNodesToRender(parentNodes, invalidNodes);
   }

   private synchronized void handleNodesRejectedThisTick(HashMap<QuadrupedFootstepPlannerNodeRejectionReason, List<FootstepNode>> rejectedNodes)
   {
      for (QuadrupedFootstepPlannerNodeRejectionReason rejectionReason : rejectedNodes.keySet())
      {
         Set<SimpleFootstepNode> invalidNodes = new HashSet<>();
         rejectedNodes.get(rejectionReason).forEach(node -> invalidNodes.add(new SimpleFootstepNode(node)));
         rejectedNodesByReasonRenderers.get(rejectionReason).processNodesToRender(invalidNodes, rejectionByReasonColor);
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

   @Override
   public void stop()
   {
      super.stop();
      executorService.shutdownNow();
   }

   public List<Node> getNodesToView()
   {
      return nodesToView;
   }
}
