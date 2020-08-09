package us.ihmc.robotEnvironmentAwareness.slam.viewer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BoundingBoxMeshView;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OccupancyMapMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.SLAMFrameStateViewer;

public class SLAMMeshViewer
{
   private static final int SLOW_PACE_UPDATE_PERIOD = 2000;
   private static final int MEDIUM_PACE_UPDATE_PERIOD = 100;
   private static final int HIGH_PACE_UPDATE_PERIOD = 50;

   private final Group root = new Group();

   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private final MeshView planarRegionMeshView = new MeshView();
   private final MeshView importedPlanarRegionMeshView = new MeshView();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(5, getClass(), ExceptionHandling.CANCEL_AND_REPORT);
   private final AnimationTimer renderMeshAnimation;

   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final PlanarRegionsMeshBuilder importedPlanarRegionsMeshBuilder;
   private final OccupancyMapMeshBuilder occupancyMapViewer;
   private final BoundingBoxMeshView boundingBoxMeshView;
   private final SLAMFrameStateViewer latestFrameStateViewer;

   private final List<AtomicReference<Boolean>> enableTopicList = new ArrayList<>();
   private final Map<AtomicReference<Boolean>, Node> enableTopicToNode = new HashMap<>();

   public SLAMMeshViewer(REAUIMessager uiMessager)
   {
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager,
                                                              SLAMModuleAPI.SLAMPlanarRegionsState,
                                                              SLAMModuleAPI.ShowPlanarRegionsMap,
                                                              SLAMModuleAPI.SLAMVizClear,
                                                              SLAMModuleAPI.SLAMClear,
                                                              SLAMModuleAPI.RequestPlanarRegions);

      importedPlanarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager,
                                                                      SLAMModuleAPI.ImportedPlanarRegionsState,
                                                                      SLAMModuleAPI.ShowImportedPlanarRegions,
                                                                      SLAMModuleAPI.SLAMVizClear,
                                                                      SLAMModuleAPI.ImportedPlanarRegionsVizClear,
                                                                      SLAMModuleAPI.RequestPlanarRegions);

      occupancyMapViewer = new OccupancyMapMeshBuilder(uiMessager);

      latestFrameStateViewer = new SLAMFrameStateViewer(uiMessager,
                                                        SLAMModuleAPI.ShowLatestFrame,
                                                        SLAMModuleAPI.SLAMVizClear,
                                                        REAModuleAPI.UIStereoVisionSize); // FIXME this is obviously the wrong topic

      boundingBoxMeshView = new BoundingBoxMeshView(uiMessager, SLAMModuleAPI.UIOcTreeBoundingBoxShow, SLAMModuleAPI.RequestBoundingBox,
                                                    SLAMModuleAPI.OcTreeBoundingBoxState);

      occupancyMapViewer.getRoot().setMouseTransparent(true);
      boundingBoxMeshView.setMouseTransparent(true);
      latestFrameStateViewer.getRoot().setMouseTransparent(true);
      root.getChildren().addAll(planarRegionMeshView, importedPlanarRegionMeshView, occupancyMapViewer.getRoot(), latestFrameStateViewer.getRoot(),
                                boundingBoxMeshView);

      addViewer(uiMessager, planarRegionMeshView, SLAMModuleAPI.ShowPlanarRegionsMap);
      addViewer(uiMessager, importedPlanarRegionMeshView, SLAMModuleAPI.ShowImportedPlanarRegions);
      addViewer(uiMessager, occupancyMapViewer.getRoot(), SLAMModuleAPI.ShowSLAMOctreeMap);
      addViewer(uiMessager, latestFrameStateViewer.getRoot(), SLAMModuleAPI.ShowLatestFrame);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            occupancyMapViewer.render();
            latestFrameStateViewer.render();

            if (planarRegionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(planarRegionMeshView, planarRegionsMeshBuilder.pollMeshAndMaterial());
            if (importedPlanarRegionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(importedPlanarRegionMeshView, importedPlanarRegionsMeshBuilder.pollMeshAndMaterial());
         }
      };

      uiMessager.registerModuleMessagerStateListener(isMessagerOpen ->
      {
         if (isMessagerOpen)
            start();
         else
            stop();
      });
   }

   private void addViewer(REAUIMessager uiMessager, Node node, Topic<Boolean> enableTopic)
   {
      AtomicReference<Boolean> enable = uiMessager.createInput(enableTopic, false);
      enableTopicToNode.put(enable, node);
      enableTopicList.add(enable);
   }

   private Runnable createViewersController()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            Platform.runLater(new Runnable()
            {
               @Override
               public void run()
               {
                  for (int i = 0; i < enableTopicList.size(); i++)
                  {
                     AtomicReference<Boolean> enable = enableTopicList.get(i);
                     Node node = enableTopicToNode.get(enable);
                     if (enable.get())
                     {
                        if (!root.getChildren().contains(node))
                           root.getChildren().addAll(node);
                     }
                     else
                     {
                        if (root.getChildren().contains(node))
                           root.getChildren().removeAll(node);
                     }
                  }
               }
            });
         }
      };
   }

   public void start()
   {
      if (!meshBuilderScheduledFutures.isEmpty())
         return;
      renderMeshAnimation.start();
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(planarRegionsMeshBuilder, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(importedPlanarRegionsMeshBuilder, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(occupancyMapViewer, 0, SLOW_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(latestFrameStateViewer, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(createViewersController(), 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(boundingBoxMeshView, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
   }

   public void sleep()
   {
      if (meshBuilderScheduledFutures.isEmpty())
         return;
      renderMeshAnimation.stop();
      for (ScheduledFuture<?> scheduledFuture : meshBuilderScheduledFutures)
         scheduledFuture.cancel(true);
      meshBuilderScheduledFutures.clear();
   }

   public void stop()
   {
      sleep();
   }

   private void updateMeshView(MeshView meshViewToUpdate, Pair<Mesh, Material> meshMaterial)
   {
      meshViewToUpdate.setMesh(meshMaterial.getKey());
      meshViewToUpdate.setMaterial(meshMaterial.getValue());
   }

   public Node getRoot()
   {
      return root;
   }
}
