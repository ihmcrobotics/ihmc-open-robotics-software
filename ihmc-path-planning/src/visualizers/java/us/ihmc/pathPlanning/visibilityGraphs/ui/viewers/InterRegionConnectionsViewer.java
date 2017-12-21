package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

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
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.InterRegionVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;

public class InterRegionConnectionsViewer extends AnimationTimer
{
   private static final boolean VERBOSE = false;

   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final MeshView connectionsMeshView = new MeshView();

   private final AtomicReference<Mesh> connectionsMeshToRender = new AtomicReference<>(null);
   private Mesh connectionsMeshRendered = null;
   private final AtomicReference<Boolean> resetRequested;
   private final AtomicReference<Boolean> show;

   public InterRegionConnectionsViewer(REAMessager messager)
   {
      this(messager, null);
   }

   public InterRegionConnectionsViewer(REAMessager messager, ExecutorService executorService)
   {
      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      connectionsMeshView.setMouseTransparent(true);
      connectionsMeshView.setMaterial(new PhongMaterial(Color.CRIMSON));

      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
      show = messager.createInput(UIVisibilityGraphsTopics.ShowLocalGraphs, false);
      messager.registerTopicListener(UIVisibilityGraphsTopics.ShowInterConnections, this::handleShowThreadSafe);
      messager.registerTopicListener(UIVisibilityGraphsTopics.InterRegionVisibilityMap, this::processInterConnectionsOnThread);
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
         connectionsMeshView.setMesh(null);
      else
         connectionsMeshView.setMesh(connectionsMeshRendered);
   }

   private void processInterConnectionsOnThread(InterRegionVisibilityMap interRegionVisibilityMap)
   {
      executorService.execute(() -> processInterConnections(interRegionVisibilityMap));
   }

   private void processInterConnections(InterRegionVisibilityMap interRegionVisibilityMap)
   {
      if (VERBOSE)
         PrintTools.info(this, "Building mesh for inter-connections.");
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Connection connection : interRegionVisibilityMap.getVisibilityMapInWorld())
         meshBuilder.addLine(connection.getSourcePoint(), connection.getTargetPoint(), VisualizationParameters.INTER_REGION_CONNECTIVITY_LINE_THICKNESS);
      connectionsMeshToRender.set(meshBuilder.generateMesh());
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         connectionsMeshView.setMesh(null);
         connectionsMeshToRender.getAndSet(null);
         return;
      }

      Mesh newMesh = connectionsMeshToRender.getAndSet(null);

      if (newMesh != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Rendering inter-connection lines.");
         connectionsMeshRendered = newMesh;
         if (show.get())
            connectionsMeshView.setMesh(newMesh);
      }
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
      return connectionsMeshView;
   }
}
