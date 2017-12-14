package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class BodyPathMeshViewer extends AnimationTimer
{
   private static final boolean VERBOSE = false;

   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final MeshView bodyPathMeshView = new MeshView();

   private final AtomicReference<Mesh> bodyPathMeshToRender = new AtomicReference<>(null);
   private Mesh bodyPathMeshRendered = null;
   private final AtomicReference<Boolean> resetRequested;

   public BodyPathMeshViewer(REAMessager messager)
   {
      this(messager, null);
   }

   public BodyPathMeshViewer(REAMessager messager, ExecutorService executorService)
   {
      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      bodyPathMeshView.setMouseTransparent(true);
      bodyPathMeshView.setMaterial(new PhongMaterial(Color.YELLOW));

      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowBodyPath, this::handleShowThreadSafe);
      messager.registerTopicListener(UIVisibilityGraphsTopics.BodyPathData, this::processBodyPathOnThread);
   }

   private void handleShowThreadSafe(boolean show)
   {
      if (Platform.isFxApplicationThread())
         handleShow(show);
      else
         Platform.runLater(() -> handleShow(show));
   }

   private void handleShow(boolean show)
   {
      if (!show)
         bodyPathMeshView.setMesh(null);
      else
         bodyPathMeshView.setMesh(bodyPathMeshRendered);
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         bodyPathMeshView.setMesh(null);
         bodyPathMeshToRender.getAndSet(null);
         return;
      }

      Mesh newMesh = bodyPathMeshToRender.getAndSet(null);

      if (newMesh != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Rendering body path line.");
         bodyPathMeshRendered = newMesh;
         bodyPathMeshView.setMesh(newMesh);
      }
   }

   private void processBodyPathOnThread(List<Point3DReadOnly> bodyPath)
   {
      executorService.execute(() -> processBodyPath(bodyPath));
   }

   private void processBodyPath(List<Point3DReadOnly> bodyPath)
   {
      if (VERBOSE)
         PrintTools.info(this, "Building mesh for body path.");
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      meshBuilder.addMultiLine(bodyPath, VisualizationParameters.BODYPATH_LINE_THICKNESS, false);
      bodyPathMeshToRender.set(meshBuilder.generateMesh());
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
      return bodyPathMeshView;
   }
}
