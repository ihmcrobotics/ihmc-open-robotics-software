package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jMonkeyEngineToolkit.tralala.Pair;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.SimpleFootstepNode;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.FootstepPathMeshViewer;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class NodeOccupancyMapSequenceRenderer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;


   private final Group root = new Group();

   private final Color validChildNodeColor;
   private final Color invalidChildNodeColor;
   private final Color parentNodeColor;

   private static final double cellWidth = 0.04;
   private static final double nodeOffsetZ = 0.05;
   private static final QuadrantDependentList<Color> pathColors = new QuadrantDependentList<>(Color.GREEN, Color.RED, Color.DARKGREEN, Color.DARKRED);

   private final List<Collection<SimpleFootstepNode>> nodesBeingExpandedBuffer = new ArrayList<>();
   private final List<Collection<SimpleFootstepNode>> validChildNodesBuffer = new ArrayList<>();
   private final List<Collection<SimpleFootstepNode>> invalidChildNodesBuffer = new ArrayList<>();
   private final List<Collection<List<SimpleFootstepNode>>> optimalPathsBuffer = new ArrayList<>();

   private final NodeOccupancyMapRenderer validChildNodeRenderer;
   private final NodeOccupancyMapRenderer invalidChildNodeRenderer;
   private final NodeOccupancyMapRenderer parentMapRenderer;

   private final AtomicReference<PlanarRegionsList> planarRegionsList;
   private final AtomicBoolean show = new AtomicBoolean();
   private final AtomicBoolean reset = new AtomicBoolean(false);
   private final AtomicBoolean resetInternal = new AtomicBoolean(false);
   private final AtomicReference<JavaFXMultiColorMeshBuilder> meshBuilderToRender = new AtomicReference<>(null);

   private final ConvexPolygon2D cellPolygon = new ConvexPolygon2D();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);


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

      planarRegionsList = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionDataTopic);

      root.getChildren().addAll(validChildNodeRenderer.getRoot(), invalidChildNodeRenderer.getRoot(), parentMapRenderer.getRoot());

      cellPolygon.addVertex(cellWidth, 0.0);
      cellPolygon.addVertex(0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-0.5 * cellWidth, 0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(-cellWidth, 0.0);
      cellPolygon.addVertex(-0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.addVertex(0.5 * cellWidth, -0.5 * Math.sqrt(3.0) * cellWidth);
      cellPolygon.update();
   }

   public void show(boolean show)
   {
      this.show.set(show);

      validChildNodeRenderer.show(show);
      invalidChildNodeRenderer.show(show);
      parentMapRenderer.show(show);

      if (show)
         meshBuilderToRender.set(meshBuilder);
   }

   public void reset()
   {
      this.reset.set(true);
      this.resetInternal.set(true);

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
      resetInternal.set(true);

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
      processFootstepPathOnThread(optimalPathsBuffer.get(frameIndex));
   }

   public void processFootstepPathOnThread(Collection<List<SimpleFootstepNode>> paths)
   {
      executorService.execute(() -> processFootstepPath(paths));
   }

   private void processFootstepPath(Collection<List<SimpleFootstepNode>> paths)
   {
      meshBuilder.clear();

      for (List<SimpleFootstepNode> path : paths)
      {
         for (SimpleFootstepNode node : path)
         {
            double x = node.getXIndex() * FootstepNode.gridSizeXY;
            double y = node.getYIndex() * FootstepNode.gridSizeXY;
            double z = getHeightAtPoint(x, y) + nodeOffsetZ;
            RigidBodyTransform transform = new RigidBodyTransform();
            transform.setTranslation(x, y, z);

            meshBuilder.addPolygon(transform, cellPolygon, pathColors.get(node.getMovingQuadrant()));
         }
      }
      meshBuilderToRender.set(meshBuilder);
   }

   private double getHeightAtPoint(double x, double y)
   {
      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      if (planarRegionsList == null)
         return 0.0;
      Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(x, y, 100.0), planarRegionsList);
      return projectedPoint == null ? 0.0 : projectedPoint.getZ();
   }

   public void processNodesToRender(Collection<SimpleFootstepNode> nodesBeingExpanded, Collection<SimpleFootstepNode> validChildNodes,
                                    Collection<SimpleFootstepNode> invalidChildNodes, Collection<List<SimpleFootstepNode>> optimalPath)
   {
      nodesBeingExpandedBuffer.add(nodesBeingExpanded);
      validChildNodesBuffer.add(validChildNodes);
      invalidChildNodesBuffer.add(invalidChildNodes);
      optimalPathsBuffer.add(optimalPath);
   }

   @Override
   public void handle(long now)
   {
      if (show.get() && meshBuilderToRender.get() != null)
      {
         JavaFXMultiColorMeshBuilder meshBuilder = meshBuilderToRender.getAndSet(null);
         MeshView meshView = new MeshView();
         meshView.setMesh(meshBuilder.generateMesh());
         meshView.setMaterial(meshBuilder.generateMaterial());

         root.getChildren().clear();
         root.getChildren().add(meshView);
         root.getChildren().addAll(validChildNodeRenderer.getRoot(), invalidChildNodeRenderer.getRoot(), parentMapRenderer.getRoot());
      }
      else if (!show.get() && !root.getChildren().isEmpty())
      {
         root.getChildren().clear();
      }

      if (resetInternal.getAndSet(false))
      {
         meshBuilder.clear();
         meshBuilderToRender.set(null);
      }

      if (reset.getAndSet(false))
      {
         validChildNodesBuffer.clear();
         invalidChildNodesBuffer.clear();
         nodesBeingExpandedBuffer.clear();
         optimalPathsBuffer.clear();
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
