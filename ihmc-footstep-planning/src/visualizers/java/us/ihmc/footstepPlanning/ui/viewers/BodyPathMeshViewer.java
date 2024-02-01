package us.ihmc.footstepPlanning.ui.viewers;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters.BODYPATH_LINE_THICKNESS;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

public class BodyPathMeshViewer extends AnimationTimer
{
   private static final boolean VERBOSE = false;

   private final Color unsmoothedColor = Color.DARKGREEN;
   private final Color smoothedColor = Color.RED;

   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Group root = new Group();
   private final MeshView bodyPathMeshView = new MeshView();

   private final AtomicReference<List<? extends Pose3DReadOnly>> activeBodyPathReference = new AtomicReference<>(null);

   private final AtomicReference<Pair<Mesh, Material>> bodyPathMeshToRender = new AtomicReference<>(null);
   private final AtomicReference<Boolean> show;
   private final AtomicBoolean reset = new AtomicBoolean(false);


   private final TextureColorAdaptivePalette palette = new TextureColorAdaptivePalette(1024, false);

   public BodyPathMeshViewer(Messager messager)
   {
      isExecutorServiceProvided = executorService == null;

      bodyPathMeshView.setMouseTransparent(true);
      bodyPathMeshView.setMaterial(new PhongMaterial(Color.YELLOW));

      Vector3D defaultSize = new Vector3D(1.0, 1.0, 1.0);
      defaultSize.scale(1.5 * BODYPATH_LINE_THICKNESS);

      show = messager.createInput(FootstepPlannerMessagerAPI.ShowBodyPath, true);
      messager.addTopicListener(FootstepPlannerMessagerAPI.BodyPathData, this::processBodyPathOnThread);

      messager.addTopicListener(FootstepPlannerMessagerAPI.ComputePath, data -> reset.set(true));
      messager.addTopicListener(FootstepPlannerMessagerAPI.GlobalReset, data -> reset.set(true));

      root.getChildren().addAll(bodyPathMeshView);
   }

   @Override
   public void handle(long now)
   {
      if (show.get() && root.getChildren().isEmpty())
      {
         root.getChildren().addAll(bodyPathMeshView);
      }
      else if (!show.get() && !root.getChildren().isEmpty())
      {
         root.getChildren().clear();
      }

      if (reset.getAndSet(false))
      {
         bodyPathMeshView.setMesh(null);
         bodyPathMeshView.setMaterial(null);
         bodyPathMeshToRender.set(null);
         return;
      }

      Pair<Mesh, Material> newMesh = bodyPathMeshToRender.getAndSet(null);
      if (newMesh != null)
      {
         if (VERBOSE)
            LogTools.info(this, "Rendering body path line.");
         bodyPathMeshView.setMesh(newMesh.getKey());
         bodyPathMeshView.setMaterial(newMesh.getValue());
      }
   }

   private void processBodyPathOnThread(org.apache.commons.lang3.tuple.Pair<List<? extends Pose3DReadOnly>, List<? extends Point3DReadOnly>> bodyPathData)
   {
      executorService.execute(() -> processBodyPath(bodyPathData));
   }

   private void processBodyPath(org.apache.commons.lang3.tuple.Pair<List<? extends Pose3DReadOnly>, List<? extends Point3DReadOnly>> bodyPathData)
   {
      if (bodyPathData == null)
      {
         return;
      }

      List<? extends Pose3DReadOnly> bodyPath = bodyPathData.getKey();
      List<? extends Point3DReadOnly> bodyPathUnsmoothed = bodyPathData.getValue();

      if (bodyPath == null || bodyPath.isEmpty())
      {
         bodyPathMeshToRender.set(new Pair<>(null, null));
         activeBodyPathReference.set(null);
         if (VERBOSE)
            LogTools.warn("Received body path that is null.");
         return;
      }

      // First let's make a deep copy for later usage.
      bodyPath = bodyPath.stream().map(Pose3D::new).collect(Collectors.toList());

      if (VERBOSE)
         LogTools.info(this, "Building mesh for body path.");

      palette.clearPalette();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(palette);

      for (int segmentIndex = 0; segmentIndex < bodyPath.size() - 1; segmentIndex++)
      {
         Point3DReadOnly lineStart = bodyPath.get(segmentIndex).getPosition();
         Point3DReadOnly lineEnd = bodyPath.get(segmentIndex + 1).getPosition();

         meshBuilder.addLine(lineStart, lineEnd, 1.5 * BODYPATH_LINE_THICKNESS, smoothedColor);
         Pose3DReadOnly waypoint = bodyPath.get(segmentIndex);
         Quaternion waypointOrientation = new Quaternion(waypoint.getYaw(), Math.toRadians(90), 0);
         meshBuilder.addCylinder(0.2, 0.01, waypoint.getPosition(), waypointOrientation, Color.PURPLE);

         if (bodyPathUnsmoothed != null && bodyPathUnsmoothed.size() > segmentIndex + 1)
         {
            lineStart = bodyPathUnsmoothed.get(segmentIndex);
            lineEnd = bodyPathUnsmoothed.get(segmentIndex + 1);
            meshBuilder.addLine(lineStart, lineEnd, 1.5 * BODYPATH_LINE_THICKNESS, unsmoothedColor);
         }
      }

      bodyPathMeshToRender.set(new Pair<>(meshBuilder.generateMesh(), meshBuilder.generateMaterial()));
      activeBodyPathReference.set(bodyPath);
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
