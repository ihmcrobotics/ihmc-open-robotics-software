package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BoundingBoxMeshView;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.BufferOctreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.LidarScanViewer;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsIntersectionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;

public class REAMeshViewer
{
   private static final int SLOW_PACE_UPDATE_PERIOD = 2000;
   private static final int MEDIUM_PACE_UPDATE_PERIOD = 100;
   private static final int HIGH_PACE_UPDATE_PERIOD = 10;

   private final Group root = new Group();

   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private final MeshView lidarBufferOcTreeMeshView = new MeshView();
   private final MeshView stereoVisionBufferOcTreeMeshView = new MeshView();
   private final MeshView depthCloudBufferOcTreeMeshView = new MeshView();
   private final MeshView planarRegionMeshView = new MeshView();
   private final MeshView intersectionsMeshView = new MeshView();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   private final AnimationTimer renderMeshAnimation;

   private final LidarScanViewer lidarScanViewer;
   private final StereoVisionPointCloudViewer stereoVisionPointCloudViewer;
   private final StereoVisionPointCloudViewer depthPointCloudViewer;
   private final BufferOctreeMeshBuilder lidarBufferOctreeMeshBuilder, stereoVisionBufferOctreeMeshBuilder, depthCloudBufferOctreeMeshBuilder;
   private final OcTreeMeshBuilder ocTreeViewer;
   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final PlanarRegionsIntersectionsMeshBuilder intersectionsMeshBuilder;
   private final BoundingBoxMeshView boundingBoxMeshView;

   public REAMeshViewer(REAUIMessager uiMessager)
   {
      lidarScanViewer = new LidarScanViewer(REAModuleAPI.LidarScanState, uiMessager, REAModuleAPI.UILidarScanShow, REAModuleAPI.UILidarScanClear);
      stereoVisionPointCloudViewer = new StereoVisionPointCloudViewer(REAModuleAPI.StereoVisionPointCloudState, uiMessager, REAModuleAPI.UIStereoVisionShow,
                                                                      REAModuleAPI.UIStereoVisionClear, REAModuleAPI.UIStereoVisionSize);
      depthPointCloudViewer = new StereoVisionPointCloudViewer(REAModuleAPI.DepthPointCloudState, uiMessager, REAModuleAPI.UIDepthCloudShow,
                                                               REAModuleAPI.UIDepthCloudClear, REAModuleAPI.UIDepthCloudSize);
      lidarBufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(uiMessager, REAModuleAPI.UIOcTreeShowLidarBuffer, REAModuleAPI.RequestLidarBuffer,
                                                                 REAModuleAPI.LidarBufferState, Color.DARKRED);
      stereoVisionBufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(uiMessager, REAModuleAPI.UIOcTreeShowStereoVisionBuffer,
                                                                        REAModuleAPI.RequestStereoVisionBuffer, REAModuleAPI.StereoVisionBufferState,
                                                                        Color.DARKORANGE);
      depthCloudBufferOctreeMeshBuilder = new BufferOctreeMeshBuilder(uiMessager, REAModuleAPI.UIOcTreeShowDepthCloudBuffer,
                                                                        REAModuleAPI.RequestDepthCloudBuffer, REAModuleAPI.DepthCloudBufferState,
                                                                        Color.DARKBLUE);
      ocTreeViewer = new OcTreeMeshBuilder(uiMessager, REAModuleAPI.OcTreeEnable, REAModuleAPI.OcTreeClear, REAModuleAPI.RequestOctree,
                                           REAModuleAPI.RequestPlanarRegionSegmentation, REAModuleAPI.UIOcTreeDepth, REAModuleAPI.UIOcTreeColoringMode,
                                           REAModuleAPI.UIOcTreeDisplayType, REAModuleAPI.UIPlanarRegionHideNodes, REAModuleAPI.OcTreeState, REAModuleAPI.PlanarRegionsSegmentationState);
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager, REAModuleAPI.PlanarRegionsState);
      intersectionsMeshBuilder = new PlanarRegionsIntersectionsMeshBuilder(uiMessager);
      boundingBoxMeshView = new BoundingBoxMeshView(uiMessager);

      Node lidarScanRootNode = lidarScanViewer.getRoot();
      lidarScanRootNode.setMouseTransparent(true);
      Node stereoVisionPointCloudRootNode = stereoVisionPointCloudViewer.getRoot();
      stereoVisionPointCloudRootNode.setMouseTransparent(true);
      Node depthPointCloudRootNode = depthPointCloudViewer.getRoot();
      depthPointCloudRootNode.setMouseTransparent(true);
      lidarBufferOcTreeMeshView.setMouseTransparent(true);
      stereoVisionBufferOcTreeMeshView.setMouseTransparent(true);
      depthCloudBufferOcTreeMeshView.setMouseTransparent(true);
      ocTreeViewer.getRoot().setMouseTransparent(true);
      boundingBoxMeshView.setMouseTransparent(true);
      root.getChildren().addAll(lidarScanRootNode, stereoVisionPointCloudRootNode, depthPointCloudRootNode, lidarBufferOcTreeMeshView,
                                stereoVisionBufferOcTreeMeshView, depthCloudBufferOcTreeMeshView, ocTreeViewer.getRoot(), planarRegionMeshView,
                                intersectionsMeshView, boundingBoxMeshView);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            lidarScanViewer.render();
            stereoVisionPointCloudViewer.render();
            depthPointCloudViewer.render();
            ocTreeViewer.render();

            if (lidarBufferOctreeMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(lidarBufferOcTreeMeshView, lidarBufferOctreeMeshBuilder.pollMeshAndMaterial());

            if (stereoVisionBufferOctreeMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(stereoVisionBufferOcTreeMeshView, stereoVisionBufferOctreeMeshBuilder.pollMeshAndMaterial());

            if (depthCloudBufferOctreeMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(depthCloudBufferOcTreeMeshView, depthCloudBufferOctreeMeshBuilder.pollMeshAndMaterial());

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
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(lidarScanViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(stereoVisionPointCloudViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(depthPointCloudViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(lidarBufferOctreeMeshBuilder, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(stereoVisionBufferOctreeMeshBuilder, 0, HIGH_PACE_UPDATE_PERIOD,
                                                                          TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(depthCloudBufferOctreeMeshBuilder, 0, HIGH_PACE_UPDATE_PERIOD,
                                                                          TimeUnit.MILLISECONDS));
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
