package us.ihmc.robotEnvironmentAwareness.slam.viewer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Material;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.util.Pair;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.SLAMOcTreeMeshBuilder;

public class IhmcSLAMMeshViewer
{
   private static final int SLOW_PACE_UPDATE_PERIOD = 2000;
   private static final int MEDIUM_PACE_UPDATE_PERIOD = 100;
   private static final int HIGH_PACE_UPDATE_PERIOD = 10;

   private final Group root = new Group();

   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private final MeshView planarRegionMeshView = new MeshView();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExceptionHandling.CANCEL_AND_REPORT);
   private final AnimationTimer renderMeshAnimation;

//   private final IhmcSLAMFrameViewer ihmcSLAMFrameViewer;
   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final OcTreeMeshBuilder ocTreeViewer;

   public IhmcSLAMMeshViewer(REAUIMessager uiMessager)
   {
//      ihmcSLAMFrameViewer = new IhmcSLAMFrameViewer(uiMessager);
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager, REAModuleAPI.SLAMPlanarRegionsState);

      ocTreeViewer = new SLAMOcTreeMeshBuilder(uiMessager, REAModuleAPI.SLAMOcTreeEnable, REAModuleAPI.SLAMOctreeMapState);

//      Node ihmcSLAMRootNode = ihmcSLAMFrameViewer.getRoot();
//      ihmcSLAMRootNode.setMouseTransparent(true);
      ocTreeViewer.getRoot().setMouseTransparent(true);
//      root.getChildren().addAll(ihmcSLAMRootNode, planarRegionMeshView, ocTreeViewer.getRoot());
      root.getChildren().addAll(planarRegionMeshView, ocTreeViewer.getRoot());

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
//            ihmcSLAMFrameViewer.render();
            ocTreeViewer.render();

            if (planarRegionsMeshBuilder.hasNewMeshAndMaterial())
               updateMeshView(planarRegionMeshView, planarRegionsMeshBuilder.pollMeshAndMaterial());
         }
      };

      uiMessager.registerModuleMessagerStateListener(isMessagerOpen -> {
         if (isMessagerOpen)
            start();
         else
            stop();
      });
   }

   public void start()
   {
      if (!meshBuilderScheduledFutures.isEmpty())
         return;
      renderMeshAnimation.start();
//      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(ihmcSLAMFrameViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(planarRegionsMeshBuilder, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(ocTreeViewer, 0, MEDIUM_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
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
