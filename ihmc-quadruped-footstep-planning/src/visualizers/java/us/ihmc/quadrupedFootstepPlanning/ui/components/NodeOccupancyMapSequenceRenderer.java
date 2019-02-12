package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.SimpleFootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class NodeOccupancyMapSequenceRenderer extends AnimationTimer
{
   private List<Node> nodesToView = new ArrayList<>();
   private ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicBoolean reset = new AtomicBoolean(false);

   private final Color childNodeColor;
   private final Color parentNodeColor;

   private final List<Collection<SimpleFootstepNode>> nodesBeingExpandedBuffer = new ArrayList<>();
   private final List<Collection<SimpleFootstepNode>> childNodesBuffer = new ArrayList<>();

   private final NodeOccupancyMapRenderer occupancyMapRenderer;
   private final NodeOccupancyMapRenderer parentMapRenderer;

   public NodeOccupancyMapSequenceRenderer(Messager messager, Color nodeBeingExpandedColor, Color childNodeColor)
   {
      this.parentNodeColor = nodeBeingExpandedColor;
      this.childNodeColor = childNodeColor;
      this.occupancyMapRenderer = new NodeOccupancyMapRenderer(messager);
      nodesToView.add(occupancyMapRenderer.getRoot());
      this.parentMapRenderer = new NodeOccupancyMapRenderer(messager);
      nodesToView.add(parentMapRenderer.getRoot());
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

      occupancyMapRenderer.processNodesToRender(childNodesBuffer.get(frameIndex), childNodeColor);
      parentMapRenderer.processNodesToRender(nodesBeingExpandedBuffer.get(frameIndex), parentNodeColor);
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
