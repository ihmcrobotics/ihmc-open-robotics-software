package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class SegmentationMeshViewer
{
   private static final int SLOW_PACE_UPDATE_PERIOD = 2000;
   private static final int MEDIUM_PACE_UPDATE_PERIOD = 100;
   private static final int HIGH_PACE_UPDATE_PERIOD = 10;

   private final Group root = new Group();

   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();

   private final MeshView planarRegionMeshView = new MeshView();
   private final MeshView intersectionsMeshView = new MeshView();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AnimationTimer renderMeshAnimation;

   private final OcTreeMeshBuilder ocTreeViewer;
   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final PlanarRegionsIntersectionsMeshBuilder intersectionsMeshBuilder;
   private final BoundingBoxMeshView boundingBoxMeshView;

   public SegmentationMeshViewer(REAUIMessager uiMessager)
   {
      ocTreeViewer = new OcTreeMeshBuilder(uiMessager,
                                           SegmentationModuleAPI.OcTreeEnable,
                                           SegmentationModuleAPI.OcTreeClear,
                                           SegmentationModuleAPI.RequestOcTree,
                                           SegmentationModuleAPI.RequestPlanarRegionSegmentation,
                                           SegmentationModuleAPI.UIOcTreeDepth,
                                           SegmentationModuleAPI.UIOcTreeColoringMode,
                                           SegmentationModuleAPI.UIOcTreeDisplayType,
                                           SegmentationModuleAPI.UIPlanarRegionHideNodes,
                                           SegmentationModuleAPI.OcTreeState,
                                           SegmentationModuleAPI.PlanarRegionsSegmentationState);
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager,
                                                              SegmentationModuleAPI.PlanarRegionsState,
                                                              SegmentationModuleAPI.OcTreeEnable,
                                                              SegmentationModuleAPI.PlanarRegionsPolygonizerClear,
                                                              SegmentationModuleAPI.OcTreeClear,
                                                              SegmentationModuleAPI.RequestPlanarRegions);
      intersectionsMeshBuilder = new PlanarRegionsIntersectionsMeshBuilder(uiMessager,
                                                                           SegmentationModuleAPI.OcTreeEnable,
                                                                           SegmentationModuleAPI.OcTreeClear,
                                                                           SegmentationModuleAPI.RequestPlanarRegionsIntersections,
                                                                           SegmentationModuleAPI.PlanarRegionsIntersectionState);

      boundingBoxMeshView = new BoundingBoxMeshView(uiMessager, SegmentationModuleAPI.UIOcTreeBoundingBoxShow, SegmentationModuleAPI.RequestBoundingBox,
                                                    SegmentationModuleAPI.OcTreeBoundingBoxState);

      ocTreeViewer.getRoot().setMouseTransparent(true);
      boundingBoxMeshView.setMouseTransparent(true);
      root.getChildren().addAll(ocTreeViewer.getRoot(), planarRegionMeshView, intersectionsMeshView, boundingBoxMeshView);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            ocTreeViewer.render();

            if (planarRegionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(planarRegionMeshView, planarRegionsMeshBuilder.pollMeshAndMaterial());

            if (intersectionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(intersectionsMeshView, intersectionsMeshBuilder.pollMeshAndMaterial());
         }
      };

      uiMessager.registerModuleMessagerStateListener(isMessagerOpen -> {
         if (isMessagerOpen)
            start();
         else
            sleep();
      });
   }

   public void start()
   {
      if (!meshBuilderScheduledFutures.isEmpty())
         return;
      renderMeshAnimation.start();
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(ocTreeViewer, 0, SLOW_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(planarRegionsMeshBuilder, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(intersectionsMeshBuilder, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
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

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
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
