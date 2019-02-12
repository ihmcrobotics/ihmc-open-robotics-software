package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedFootstepPlanning.ui.SimpleFootstepNode;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

public class NodeOccupancyMapSequenceRenderer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final Group root = new Group();

   private final Color childNodeColor;
   private final Color parentNodeColor;

   private final List<Collection<SimpleFootstepNode>> nodesBeingExpandedBuffer = new ArrayList<>();
   private final List<Collection<SimpleFootstepNode>> childNodesBuffer = new ArrayList<>();

   private final NodeOccupancyMapRenderer occupancyMapRenderer;
   private final NodeOccupancyMapRenderer parentMapRenderer;

   public NodeOccupancyMapSequenceRenderer(Messager messager, Color nodeBeingExpandedColor, Color childNodeColor, ExecutorService executorService)
   {
      this.parentNodeColor = nodeBeingExpandedColor;
      this.childNodeColor = childNodeColor;

      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      this.occupancyMapRenderer = new NodeOccupancyMapRenderer(messager, executorService);
      this.parentMapRenderer = new NodeOccupancyMapRenderer(messager, executorService);

      root.getChildren().addAll(occupancyMapRenderer.getRoot(), parentMapRenderer.getRoot());
   }

   public void show(boolean show)
   {
      occupancyMapRenderer.show(show);
      parentMapRenderer.show(show);
   }

   public void reset()
   {
      occupancyMapRenderer.reset();
      parentMapRenderer.reset();
   }

   public void requestSpecificPercentageInPlayback(double alpha)
   {
      if (nodesBeingExpandedBuffer.size() != childNodesBuffer.size())
         throw new RuntimeException("The buffers are not the same size.");

      int size = nodesBeingExpandedBuffer.size();
      int frameIndex = (int) (alpha * (size - 1));
      setToFrame(frameIndex);
   }

   private void setToFrame(int frameIndex)
   {
      occupancyMapRenderer.reset();
      parentMapRenderer.reset();

      occupancyMapRenderer.processNodesToRenderOnThread(childNodesBuffer.get(frameIndex), childNodeColor);
      parentMapRenderer.processNodesToRenderOnThread(nodesBeingExpandedBuffer.get(frameIndex), parentNodeColor);
   }

   public void processNodesToRender(Collection<SimpleFootstepNode> nodesBeingExpanded, Collection<SimpleFootstepNode> childNodes)
   {
      nodesBeingExpandedBuffer.add(nodesBeingExpanded);
      childNodesBuffer.add(childNodes);
   }

   @Override
   public void handle(long now)
   {
      if (reset.getAndSet(false))
      {
         childNodesBuffer.clear();
         nodesBeingExpandedBuffer.clear();
         return;
      }

   }

   @Override
   public void start()
   {
      super.start();

      occupancyMapRenderer.start();
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

      occupancyMapRenderer.stop();
      parentMapRenderer.stop();
   }

   public Node getRoot()
   {
      return root;
   }
}
