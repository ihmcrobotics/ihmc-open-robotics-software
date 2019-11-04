package us.ihmc.quadrupedFootstepPlanning.ui.components;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.PawPathMeshViewer;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class NodeOccupancyMapSequenceRenderer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private static final double coneHeight = 0.15;
   private static final double coneRadius = 0.03;

   private final Group root = new Group();

   private final Color validChildNodeColor;
   private final Color invalidChildNodeColor;
   private final Color parentNodeColor;

   private static final double cellWidth = 0.04;
   private static final double nodeOffsetZ = 0.05;
   private static final QuadrantDependentList<Color> pathColors = new QuadrantDependentList<>(Color.GREEN, Color.RED, Color.DARKGREEN, Color.DARKRED);

   private final List<Collection<PawNode>> nodesBeingExpandedBuffer = new ArrayList<>();
   private final List<Collection<PawNode>> validChildNodesBuffer = new ArrayList<>();
   private final List<Collection<PawNode>> invalidChildNodesBuffer = new ArrayList<>();
   private final List<Collection<List<PawNode>>> optimalPathsBuffer = new ArrayList<>();

   private final NodeOccupancyMapRenderer validChildNodeRenderer;
   private final NodeOccupancyMapRenderer invalidChildNodeRenderer;
   private final NodeOccupancyMapRenderer parentMapRenderer;

   private final AtomicReference<PlanarRegionsList> planarRegionsList;
   private final AtomicBoolean show = new AtomicBoolean();
   private final AtomicBoolean reset = new AtomicBoolean(false);
   private final AtomicBoolean resetInternal = new AtomicBoolean(false);

   private final AtomicReference<JavaFXMultiColorMeshBuilder> pathMeshBuilderToRender = new AtomicReference<>(null);
   private final AtomicReference<JavaFXMultiColorMeshBuilder> parentFeetMeshBuilderToRender = new AtomicReference<>(null);
   private final AtomicReference<List<Group>> parentCenterMeshToRender = new AtomicReference<>(null);

   private final ConvexPolygon2D cellPolygon = new ConvexPolygon2D();

   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);
   private final JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final JavaFXMultiColorMeshBuilder parentFeetMeshBuilder = new JavaFXMultiColorMeshBuilder(palette);
   private final List<Group> centerMesh = new ArrayList<>();


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

      planarRegionsList = messager.createInput(PawStepPlannerMessagerAPI.PlanarRegionDataTopic);

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
      {
         pathMeshBuilderToRender.set(meshBuilder);
         parentFeetMeshBuilderToRender.set(parentFeetMeshBuilder);
         parentCenterMeshToRender.set(centerMesh);
      }
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

      Collection<PawNode> nodesBeingExpanded = nodesBeingExpandedBuffer.get(frameIndex);
      Color parentNodeColor = this.parentNodeColor;
      if (nodesBeingExpanded != null && !nodesBeingExpanded.isEmpty())
      {
         for (PawNode node : nodesBeingExpanded)
         {
            parentNodeColor = PawPathMeshViewer.defaultSolutionFootstepColors.get(node.getMovingQuadrant());
            break;
         }
      }

      validChildNodeRenderer.processNodesToRenderOnThread(validChildNodesBuffer.get(frameIndex), validChildNodeColor);
      invalidChildNodeRenderer.processNodesToRenderOnThread(invalidChildNodesBuffer.get(frameIndex), invalidChildNodeColor);
      parentMapRenderer.processNodesToRenderOnThread(nodesBeingExpanded, parentNodeColor);
      processFootstepPathOnThread(optimalPathsBuffer.get(frameIndex));
      processParentNodesOnThread(nodesBeingExpanded);
   }

   private void processFootstepPathOnThread(Collection<List<PawNode>> paths)
   {
      executorService.execute(() -> processFootstepPath(paths));
   }

   private void processFootstepPath(Collection<List<PawNode>> paths)
   {
      meshBuilder.clear();

      for (List<PawNode> path : paths)
      {
         for (PawNode node : path)
         {
            RobotQuadrant movingQuadrant = node.getMovingQuadrant();
            double x = node.getXIndex(movingQuadrant) * PawNode.gridSizeXY;
            double y = node.getYIndex(movingQuadrant) * PawNode.gridSizeXY;
            double z = getHeightAtPoint(x, y) + nodeOffsetZ;
            RigidBodyTransform transform = new RigidBodyTransform();
            transform.setTranslation(x, y, z);

            meshBuilder.addPolygon(transform, cellPolygon, pathColors.get(movingQuadrant));
         }
      }
      pathMeshBuilderToRender.set(meshBuilder);
   }

   private void processParentNodesOnThread(Collection<PawNode> parentNodes)
   {
      executorService.execute(() -> processParentNodes(parentNodes));
   }

   private void processParentNodes(Collection<PawNode> parentNodes)
   {
      parentFeetMeshBuilder.clear();
      centerMesh.clear();

      for (PawNode parentNode : parentNodes)
      {
         Point2DReadOnly centerPosition2D = parentNode.getOrComputeXGaitCenterPoint();
         double centerHeight = getHeightAtPoint(centerPosition2D.getX(), centerPosition2D.getY()) + nodeOffsetZ;

         Point3DReadOnly centerPosition = new Point3D(centerPosition2D.getX(), centerPosition2D.getY(), centerHeight);
         double yaw = parentNode.getStepYaw();
         double cylinderLength = 5.0 * coneRadius;
         ArrowGraphic orientationArrow = new ArrowGraphic(0.2 * coneRadius, cylinderLength, Color.GREEN);

         Quaternion orientation = new Quaternion();
         orientation.setToYawQuaternion(yaw);
         PoseReferenceFrame xGaitFrame = new PoseReferenceFrame("xGaitFrame", ReferenceFrame.getWorldFrame());
         xGaitFrame.setPoseAndUpdate(centerPosition, orientation);

         orientationArrow.setTranslateX(centerPosition.getX() + 0.5 * cylinderLength * (Math.cos(yaw) - 1.0));
         orientationArrow.setTranslateY(centerPosition.getY() + 0.5 * cylinderLength * Math.sin(yaw));
         orientationArrow.setTranslateZ(centerPosition.getZ());
         orientationArrow.setRotate(Math.toDegrees(yaw));
         centerMesh.add(orientationArrow);

         parentFeetMeshBuilder.addSphere(coneRadius, centerPosition, Color.GREEN);

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            Color color;
            if (robotQuadrant == parentNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant())
               color = Color.DARKCYAN;
            else
               color = pathColors.get(robotQuadrant);

            double x = parentNode.getXIndex(robotQuadrant) * PawNode.gridSizeXY;
            double y = parentNode.getYIndex(robotQuadrant) * PawNode.gridSizeXY;
            double z = getHeightAtPoint(x, y) + nodeOffsetZ;
            RigidBodyTransform transform = new RigidBodyTransform();
            transform.setTranslation(x, y, z);

            FramePoint3D xGaitFoot = new FramePoint3D(xGaitFrame, 0.5 * robotQuadrant.getEnd().negateIfHindEnd(parentNode.getNominalStanceLength()),
                                                      0.5 * robotQuadrant.getSide().negateIfRightSide(parentNode.getNominalStanceWidth()), 0.0);
            xGaitFoot.changeFrame(ReferenceFrame.getWorldFrame());
            xGaitFoot.setZ(getHeightAtPoint(xGaitFoot.getX(), xGaitFoot.getY()) + nodeOffsetZ);

            parentFeetMeshBuilder.addCone(coneHeight, coneRadius, transform.getTranslationVector(), color);
            parentFeetMeshBuilder.addCone(0.5 * coneHeight, coneRadius, xGaitFoot, Color.LIGHTBLUE);
         }


      }

      parentFeetMeshBuilderToRender.set(parentFeetMeshBuilder);
      parentCenterMeshToRender.set(centerMesh);
   }

   private double getHeightAtPoint(double x, double y)
   {
      PlanarRegionsList planarRegionsList = this.planarRegionsList.get();
      if (planarRegionsList == null)
         return 0.0;
      Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(x, y, 100.0), planarRegionsList);
      return projectedPoint == null ? 0.0 : projectedPoint.getZ();
   }

   public void processNodesToRender(Collection<PawNode> nodesBeingExpanded, Collection<PawNode> validChildNodes,
                                    Collection<PawNode> invalidChildNodes, Collection<List<PawNode>> optimalPath)
   {
      nodesBeingExpandedBuffer.add(nodesBeingExpanded);
      validChildNodesBuffer.add(validChildNodes);
      invalidChildNodesBuffer.add(invalidChildNodes);
      optimalPathsBuffer.add(optimalPath);
   }

   @Override
   public void handle(long now)
   {
      if (show.get() && (parentFeetMeshBuilderToRender.get() != null || pathMeshBuilderToRender.get() != null))
      {
         root.getChildren().clear();

         if (pathMeshBuilderToRender.get() != null)
         {
            JavaFXMultiColorMeshBuilder pathMeshBuilder = pathMeshBuilderToRender.getAndSet(null);
            MeshView pathMeshView = new MeshView();
            pathMeshView.setMesh(pathMeshBuilder.generateMesh());
            pathMeshView.setMaterial(pathMeshBuilder.generateMaterial());
            root.getChildren().add(pathMeshView);
         }

         if (parentFeetMeshBuilderToRender.get() != null)
         {
            JavaFXMultiColorMeshBuilder parentMeshBuilder = parentFeetMeshBuilderToRender.getAndSet(null);
            MeshView parentMeshView = new MeshView();
            parentMeshView.setMesh(parentMeshBuilder.generateMesh());
            parentMeshView.setMaterial(parentMeshBuilder.generateMaterial());
            root.getChildren().add(parentMeshView);
            root.getChildren().addAll(parentCenterMeshToRender.getAndSet(null));
         }

         root.getChildren().addAll(validChildNodeRenderer.getRoot(), invalidChildNodeRenderer.getRoot(), parentMapRenderer.getRoot());
      }
      else if (!show.get() && !root.getChildren().isEmpty())
      {
         root.getChildren().clear();
      }

      if (resetInternal.getAndSet(false))
      {
         meshBuilder.clear();
         parentFeetMeshBuilder.clear();
         centerMesh.clear();
         pathMeshBuilderToRender.set(null);
         parentFeetMeshBuilderToRender.set(null);
         parentCenterMeshToRender.set(null);
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

   private class ArrowGraphic extends Group
   {
      private final MeshView arrow;

      public ArrowGraphic(double radius, double length, Color color)
      {
         TextureColorPalette1D colorPalette = new TextureColorPalette1D();
         colorPalette.setHueBased(1.0, 1.0);
         JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

         double coneHeight = 0.10 * length;
         double coneRadius = 1.5 * radius;

         meshBuilder.addCylinder(length, radius, new Point3D(), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);
         meshBuilder.addCone(coneHeight, coneRadius, new Point3D(length, 0.0, 0.0), new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), color);

         this.arrow = new MeshView(meshBuilder.generateMesh());
         arrow.setMaterial(meshBuilder.generateMaterial());
         getChildren().add(arrow);
      }

      public void setMaterial(PhongMaterial material)
      {
         arrow.setMaterial(material);
      }
   }
}
