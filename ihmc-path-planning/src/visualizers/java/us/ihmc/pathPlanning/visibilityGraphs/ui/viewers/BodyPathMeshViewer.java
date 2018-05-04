package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters.BODYPATH_LINE_THICKNESS;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import com.google.common.util.concurrent.AtomicDouble;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.util.Pair;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PathTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class BodyPathMeshViewer extends AnimationTimer
{
   private static final boolean VERBOSE = false;

   private static final double WALKER_SPEED = 0.01;

   private final double startColorHue = Color.GREEN.getHue();
   private final double goalColorHue = Color.RED.getHue();

   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final Group root = new Group();
   private final MeshView bodyPathMeshView = new MeshView();
   private final Sphere walker = new Sphere();

   private final AtomicDouble currentWalkerDistanceInPath = new AtomicDouble(0.0);
   private final AtomicReference<List<Point3DReadOnly>> activeBodyPathReference = new AtomicReference<>(null);

   private final AtomicReference<Pair<Mesh, Material>> bodyPathMeshToRender = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show, resetRequested;
   private final AtomicReference<Vector3D> walkerSize;
   private final AtomicReference<Double> walkerOffsetHeight;
   private final AtomicReference<Point3D> walkerPosition;
   private final AtomicReference<Boolean> enableWalkerAnimation;
   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);

   public BodyPathMeshViewer(Messager messager)
   {
      this(messager, null);
   }

   public BodyPathMeshViewer(Messager messager, ExecutorService executorService)
   {
      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      bodyPathMeshView.setMouseTransparent(true);
      bodyPathMeshView.setMaterial(new PhongMaterial(Color.YELLOW));

      Vector3D defaultSize = new Vector3D(1.0, 1.0, 1.0);
      defaultSize.scale(1.5 * BODYPATH_LINE_THICKNESS);
      walkerSize = messager.createInput(UIVisibilityGraphsTopics.WalkerSize, defaultSize);
      walkerOffsetHeight = messager.createInput(UIVisibilityGraphsTopics.WalkerOffsetHeight, 0.0);
      walker.setMaterial(new PhongMaterial(Color.YELLOW));
      walker.setRadius(1.0);

      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      show = messager.createInput(UIVisibilityGraphsTopics.ShowBodyPath, true);
      messager.registerTopicListener(UIVisibilityGraphsTopics.BodyPathData, this::processBodyPathOnThread);

      walkerPosition = messager.createInput(UIVisibilityGraphsTopics.WalkerPosition, null);
      enableWalkerAnimation = messager.createInput(UIVisibilityGraphsTopics.EnableWalkerAnimation, true);

      root.getChildren().addAll(bodyPathMeshView, walker);
   }

   @Override
   public void handle(long now)
   {
      if (show.get() && root.getChildren().isEmpty())
      {
         root.getChildren().addAll(bodyPathMeshView, walker);
         currentWalkerDistanceInPath.set(0.0);
      }
      else if (!show.get() && !root.getChildren().isEmpty())
      {
         root.getChildren().clear();
      }

      if (resetRequested.getAndSet(false))
      {
         bodyPathMeshView.setMesh(null);
         bodyPathMeshToRender.getAndSet(null);
         currentWalkerDistanceInPath.set(0.0);
         activeBodyPathReference.set(null);
         return;
      }

      Pair<Mesh, Material> newMesh = bodyPathMeshToRender.get();

      if (newMesh != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Rendering body path line.");
         bodyPathMeshView.setMesh(newMesh.getKey());
         bodyPathMeshView.setMaterial(newMesh.getValue());
      }

      walker.setScaleX(walkerSize.get().getX());
      walker.setScaleY(walkerSize.get().getY());
      walker.setScaleZ(walkerSize.get().getZ());

      if (enableWalkerAnimation.get())
      {
         List<Point3DReadOnly> bodyPath = activeBodyPathReference.get();

         if (bodyPath != null)
         {
            if (show.get() && !root.getChildren().contains(walker))
               root.getChildren().add(walker);

            double distance = currentWalkerDistanceInPath.getAndAdd(WALKER_SPEED);
            setWalkerPosition(PathTools.getPointAlongPathGivenDistanceFromStart(bodyPath, distance));

            if (currentWalkerDistanceInPath.get() > PathTools.computePathLength(bodyPath))
               currentWalkerDistanceInPath.set(0.0);
         }
         else
         {
            root.getChildren().remove(walker);
         }
      }
      else
      {
         Point3D position = walkerPosition.get();
         if (position != null)
         {
            if (show.get() && !root.getChildren().contains(walker))
               root.getChildren().add(walker);

            setWalkerPosition(position);
         }
         else
         {
            root.getChildren().remove(walker);
         }
      }

   }

   private void setWalkerPosition(Point3DReadOnly position)
   {
      if (position == null)
         return;
      walker.setTranslateX(position.getX());
      walker.setTranslateY(position.getY());
      walker.setTranslateZ(position.getZ() + walkerOffsetHeight.get());
   }

   private void processBodyPathOnThread(List<Point3DReadOnly> bodyPath)
   {
      executorService.execute(() -> processBodyPath(bodyPath));
   }

   private void processBodyPath(List<Point3DReadOnly> bodyPath)
   {
      if (bodyPath == null || bodyPath.isEmpty())
      {
         bodyPathMeshToRender.set(new Pair<>(null, null));
         activeBodyPathReference.set(null);
         PrintTools.warn("Received body path that is null.");
         return;
      }

      // First let's make a deep copy for later usage.
      bodyPath = bodyPath.stream().map(Point3D::new).collect(Collectors.toList());

      if (VERBOSE)
         PrintTools.info(this, "Building mesh for body path.");

      double totalPathLength = PathTools.computePathLength(bodyPath);
      double currentLength = 0.0;

      palette.clearPalette();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

      for (int segmentIndex = 0; segmentIndex < bodyPath.size() - 1; segmentIndex++)
      {
         Point3DReadOnly lineStart = bodyPath.get(segmentIndex);
         Point3DReadOnly lineEnd = bodyPath.get(segmentIndex + 1);

         double lineStartHue = EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         currentLength += lineStart.distance(lineEnd);
         double lineEndHue = EuclidCoreTools.interpolate(startColorHue, goalColorHue, currentLength / totalPathLength);
         meshBuilder.addLine(lineStart, lineEnd, BODYPATH_LINE_THICKNESS, Color.hsb(lineStartHue, 1.0, 0.5), Color.hsb(lineEndHue, 1.0, 1.0));
      }
      bodyPathMeshToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      activeBodyPathReference.set(bodyPath);
      currentWalkerDistanceInPath.set(0.0);
   }

   @Override
   public void stop()
   {
      super.stop();

      if (!isExecutorServiceProvided)
         executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}
