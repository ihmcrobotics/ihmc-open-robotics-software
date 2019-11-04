package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXVisualizers.IdMappedColorFunction;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class VisibilityMapHolderViewer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final Group root = new Group();
   private final ObservableList<Node> rootChildren = root.getChildren();
   private Material customMaterial = null;

   private final AtomicReference<MeshView> mapToRender = new AtomicReference<>(null);

   private AtomicReference<Boolean> resetRequested;
   private AtomicReference<Boolean> show;
   private AtomicReference<? extends VisibilityMapHolder> newDataReference;
   private final Messager messager;

   public VisibilityMapHolderViewer(Messager messager)
   {
      this(messager, null);
   }

   public VisibilityMapHolderViewer(Messager messager, ExecutorService executorService)
   {
      this.messager = messager;
      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      resetRequested = messager.createInput(UIVisibilityGraphsTopics.GlobalReset, false);
   }

   public void setTopics(Topic<Boolean> showTopic, Topic<? extends VisibilityMapHolder> dataTopic)
   {
      show = messager.createInput(showTopic, false);
      newDataReference = messager.createInput(dataTopic, null);
   }

   public void setCustomColor(Color color)
   {
      customMaterial = new PhongMaterial(color);
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         rootChildren.clear();
         mapToRender.getAndSet(null);
         return;
      }

      if (show.get())
      {
         MeshView newMeshView = mapToRender.get();

         rootChildren.clear();
         if (newMeshView != null)
            rootChildren.add(newMeshView);

         VisibilityMapHolder newData = newDataReference.getAndSet(null);
         if (newData != null)
            processMapOnThread(newData);
      }
      else if (!rootChildren.isEmpty())
      {
         rootChildren.clear();
      }
   }

   private void processMapOnThread(VisibilityMapHolder visibilityMapHolder)
   {
      executorService.execute(() -> processMap(visibilityMapHolder));
   }

   private void processMap(VisibilityMapHolder visibilityMapHolder)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Connection connection : visibilityMapHolder.getVisibilityMapInWorld())
         meshBuilder.addLine(connection.getSourcePoint(), connection.getTargetPoint(), VisualizationParameters.INTER_REGION_CONNECTIVITY_LINE_THICKNESS);

      MeshView meshView = new MeshView(meshBuilder.generateMesh());

      if (customMaterial != null)
         meshView.setMaterial(customMaterial);
      else
         meshView.setMaterial(new PhongMaterial(IdMappedColorFunction.INSTANCE.apply(visibilityMapHolder.getMapId())));

      mapToRender.set(meshView);
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
