package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.ui.SimpleFootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.FootstepPathMeshViewer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

public class NodeOccupancyMapSequenceRenderer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final Group root = new Group();

   private final Color validChildNodeColor;
   private final Color invalidChildNodeColor;
   private final Color parentNodeColor;

   private final List<Collection<SimpleFootstepNode>> nodesBeingExpandedBuffer = new ArrayList<>();
   private final List<Collection<SimpleFootstepNode>> validChildNodesBuffer = new ArrayList<>();
   private final List<Collection<SimpleFootstepNode>> invalidChildNodesBuffer = new ArrayList<>();

   private final NodeOccupancyMapRenderer validChildNodeRenderer;
   private final NodeOccupancyMapRenderer invalidChildNodeRenderer;
   private final NodeOccupancyMapRenderer parentMapRenderer;

   public NodeOccupancyMapSequenceRenderer(Messager messager, Color nodeBeingExpandedColor, Color validChildNodeColor, Color invalidChildNodeColor,
                                           ExecutorService executorService)
   {
      this.parentNodeColor = nodeBeingExpandedColor;
      this.validChildNodeColor = validChildNodeColor;
      this.invalidChildNodeColor = invalidChildNodeColor;

      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      this.validChildNodeRenderer = new NodeOccupancyMapRenderer(messager, executorService);
      this.invalidChildNodeRenderer = new NodeOccupancyMapRenderer(messager, executorService);
      this.parentMapRenderer = new NodeOccupancyMapRenderer(messager, executorService);

      root.getChildren().addAll(validChildNodeRenderer.getRoot(), invalidChildNodeRenderer.getRoot(), parentMapRenderer.getRoot());
   }

   public void show(boolean show)
   {
      validChildNodeRenderer.show(show);
      invalidChildNodeRenderer.show(show);
      parentMapRenderer.show(show);
   }

   public void reset()
   {
      validChildNodeRenderer.reset();
      invalidChildNodeRenderer.reset();
      parentMapRenderer.reset();
   }

   public void requestSpecificPercentageInPlayback(double alpha)
   {
      if (nodesBeingExpandedBuffer.size() != validChildNodesBuffer.size() && nodesBeingExpandedBuffer.size() != invalidChildNodesBuffer.size())
         throw new RuntimeException("The buffers are not the same size.");

      int size = nodesBeingExpandedBuffer.size();
      int frameIndex = (int) (alpha * (size - 1));
      setToFrame(frameIndex);
   }

   private void setToFrame(int frameIndex)
   {
      validChildNodeRenderer.reset();
      invalidChildNodeRenderer.reset();
      parentMapRenderer.reset();

      if (nodesBeingExpandedBuffer.size() < 1)
         return;

      Collection<SimpleFootstepNode> nodesBeingExpanded = nodesBeingExpandedBuffer.get(frameIndex);
      Color parentNodeColor = this.parentNodeColor;
      if (nodesBeingExpanded != null && !nodesBeingExpanded.isEmpty())
      {
         for (SimpleFootstepNode node : nodesBeingExpanded)
         {
            parentNodeColor = FootstepPathMeshViewer.solutionFootstepColors.get(node.getMovingQuadrant());
            break;
         }
      }

      validChildNodeRenderer.processNodesToRenderOnThread(validChildNodesBuffer.get(frameIndex), validChildNodeColor);
      invalidChildNodeRenderer.processNodesToRenderOnThread(invalidChildNodesBuffer.get(frameIndex), invalidChildNodeColor);
      parentMapRenderer.processNodesToRenderOnThread(nodesBeingExpanded, parentNodeColor);
   }

   public void processNodesToRender(Collection<SimpleFootstepNode> nodesBeingExpanded, Collection<SimpleFootstepNode> validChildNodes, Collection<SimpleFootstepNode> invalidChildNodes)
   {
      nodesBeingExpandedBuffer.add(nodesBeingExpanded);
      validChildNodesBuffer.add(validChildNodes);
      invalidChildNodesBuffer.add(invalidChildNodes);
   }

   @Override
   public void handle(long now)
   {
      if (reset.getAndSet(false))
      {
         validChildNodesBuffer.clear();
         invalidChildNodesBuffer.clear();
         nodesBeingExpandedBuffer.clear();
      }
   }

   @Override
   public void start()
   {
      super.start();

      validChildNodeRenderer.start();
      invalidChildNodeRenderer.start();
      parentMapRenderer.start();
   }

   @Override
   public void stop()
   {
      try
      {
         super.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      if (!isExecutorServiceProvided)
         executorService.shutdownNow();

      validChildNodeRenderer.stop();
      invalidChildNodeRenderer.stop();
      parentMapRenderer.stop();
   }

   public Node getRoot()
   {
      return root;
   }
}
