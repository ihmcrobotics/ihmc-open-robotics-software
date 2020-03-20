package us.ihmc.robotEnvironmentAwareness.slam.viewer;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
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
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.PlanarRegionsMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.SLAMOcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.SensorFrameViewer;

public class SLAMMeshViewer
{
   private static final int UPDATE_PERIOD = 100;

   private final Group root = new Group();

   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private final MeshView planarRegionMeshView = new MeshView();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(4, getClass(), ExceptionHandling.CANCEL_AND_REPORT);
   private final AnimationTimer renderMeshAnimation;

   private final PlanarRegionsMeshBuilder planarRegionsMeshBuilder;
   private final SLAMOcTreeMeshBuilder ocTreeViewer;
   private final StereoVisionPointCloudViewer latestBufferViewer;
   private final SensorFrameViewer<StereoVisionPointCloudMessage> sensorFrameViewer;

   private final List<AtomicReference<Boolean>> enableTopicList = new ArrayList<>();
   private final Map<AtomicReference<Boolean>, Node> enableTopicToNode = new HashMap<>();

   public SLAMMeshViewer(REAUIMessager uiMessager)
   {
      planarRegionsMeshBuilder = new PlanarRegionsMeshBuilder(uiMessager, REAModuleAPI.SLAMPlanarRegionsState, REAModuleAPI.ShowPlanarRegionsMap,
                                                              REAModuleAPI.SLAMVizClear, REAModuleAPI.SLAMClear);

      ocTreeViewer = new SLAMOcTreeMeshBuilder(uiMessager, REAModuleAPI.ShowSLAMOctreeMap, REAModuleAPI.SLAMClear, REAModuleAPI.SLAMOctreeMapState,
                                               REAModuleAPI.SLAMOcTreeDisplayType);

      latestBufferViewer = new StereoVisionPointCloudViewer(REAModuleAPI.IhmcSLAMFrameState, uiMessager, REAModuleAPI.ShowLatestFrame,
                                                            REAModuleAPI.SLAMVizClear);

      sensorFrameViewer = new SensorFrameViewer<StereoVisionPointCloudMessage>(uiMessager, REAModuleAPI.IhmcSLAMFrameState,
                                                                               REAModuleAPI.UISensorPoseHistoryFrames,
                                                                               SensorFrameViewer.createStereoVisionSensorFrameExtractor());

      ocTreeViewer.getRoot().setMouseTransparent(true);
      latestBufferViewer.getRoot().setMouseTransparent(true);
      sensorFrameViewer.getRoot().setMouseTransparent(true);
      root.getChildren().addAll(planarRegionMeshView, ocTreeViewer.getRoot(), latestBufferViewer.getRoot(), sensorFrameViewer.getRoot());

      addViewer(uiMessager, planarRegionMeshView, REAModuleAPI.ShowPlanarRegionsMap);
      addViewer(uiMessager, ocTreeViewer.getRoot(), REAModuleAPI.ShowSLAMOctreeMap);
      addViewer(uiMessager, latestBufferViewer.getRoot(), REAModuleAPI.ShowLatestFrame);
      addViewer(uiMessager, sensorFrameViewer.getRoot(), REAModuleAPI.ShowSLAMSensorTrajectory);

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            ocTreeViewer.render();
            latestBufferViewer.render();

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
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(planarRegionsMeshBuilder, 0, UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(ocTreeViewer, 0, UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(latestBufferViewer, 0, UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(createViewersController(), 0, UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      sensorFrameViewer.start();
   }

   public void sleep()
   {
      if (meshBuilderScheduledFutures.isEmpty())
         return;
      renderMeshAnimation.stop();
      sensorFrameViewer.stop();
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
