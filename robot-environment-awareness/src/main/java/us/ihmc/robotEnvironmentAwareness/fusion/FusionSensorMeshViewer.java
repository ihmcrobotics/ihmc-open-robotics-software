package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import boofcv.struct.calib.IntrinsicParameters;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.fusion.objectDetection.DetectedObjectViewer;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.LidarScanViewer;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;
import us.ihmc.ros2.Ros2Node;

public class FusionSensorMeshViewer
{
   private static final int HIGH_PACE_UPDATE_PERIOD = 10;

   private final Group root = new Group();

   private final LidarScanViewer lidarScanViewer;
   private final StereoVisionPointCloudViewer stereoVisionPointCloudViewer;
   private final DetectedObjectViewer detectedObjectViewer;

   private final AnimationTimer renderMeshAnimation;
   private final List<ScheduledFuture<?>> meshBuilderScheduledFutures = new ArrayList<>();
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CANCEL_AND_REPORT);
   
   private static final String pointCloudDataFileName = "C:\\Users\\inhol\\Desktop\\SavedData\\stereovisionpointcloud.txt";
   private static final String labeledImageDataFileName = "C:\\Users\\inhol\\Desktop\\SavedData\\labeledimage.txt";
   private static final int imageWidth = 1024;
   private static final int imageHeight = 544;

   private final IntrinsicParameters intrinsicParameters = PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters;
   private final LidarImageFusionRawDataLoader dataLoader = new LidarImageFusionRawDataLoader();

   public FusionSensorMeshViewer(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager, REAUIMessager reaMessager) throws Exception
   {
      lidarScanViewer = new LidarScanViewer(REAModuleAPI.LidarScanState, reaMessager);
      stereoVisionPointCloudViewer = new StereoVisionPointCloudViewer(REAModuleAPI.StereoVisionPointCloudState, reaMessager);
      detectedObjectViewer = new DetectedObjectViewer(ros2Node);

      Node lidarScanRootNode = lidarScanViewer.getRoot();
      lidarScanRootNode.setMouseTransparent(true);
      Node stereoVisionPointCloudRootNode = stereoVisionPointCloudViewer.getRoot();
      stereoVisionPointCloudRootNode.setMouseTransparent(true);
      Node detectedObjectRootNode = detectedObjectViewer.getRoot();
      detectedObjectRootNode.setMouseTransparent(true);

      root.getChildren().addAll(lidarScanRootNode, stereoVisionPointCloudRootNode, detectedObjectRootNode);

      //messager.registerTopicListener(LidarImageFusionAPI.ClearObjects, (content) -> detectedObjectViewer.clear());
      messager.registerTopicListener(LidarImageFusionAPI.ClearObjects, (content) -> tempVisualize());

      renderMeshAnimation = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            lidarScanViewer.render();
            stereoVisionPointCloudViewer.render();
            detectedObjectViewer.render();
         }
      };
      start();
   }

   private void tempVisualize()
   {
      dataLoader.loadLidarImageFusionRawData("dataOne", pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
      LidarImageFusionRawData rawData = dataLoader.getRawData("dataOne");
      int numberOfLabels = rawData.getNumberOfLabels();
      
      for(int i=0;i<numberOfLabels;i++)
      {
         Point3D labelCenter = rawData.getLabelCenter(i);
         Vector3D labelNormal = rawData.getLabelNormal(i);
         Point3D labelNormalEnd = new Point3D(labelNormal);
         labelNormalEnd.scaleAdd(0.05, labelCenter);
         
         JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();
         meshBuilder.addSphere(0.01, labelCenter, Color.PINK);
         meshBuilder.addLine(labelCenter, labelNormalEnd, 0.01, Color.RED);
         MeshView newScanMeshView = new MeshView(meshBuilder.generateMesh());
         newScanMeshView.setMaterial(meshBuilder.generateMaterial());
         
         root.getChildren().add(newScanMeshView);
      }
   }

   public void start()
   {
      renderMeshAnimation.start();

      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(lidarScanViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
      meshBuilderScheduledFutures.add(executorService.scheduleAtFixedRate(stereoVisionPointCloudViewer, 0, HIGH_PACE_UPDATE_PERIOD, TimeUnit.MILLISECONDS));
   }

   public void sleep()
   {
      renderMeshAnimation.stop();
      meshBuilderScheduledFutures.clear();
   }

   public void stop()
   {
      sleep();
   }

   public Node getRoot()
   {
      return root;
   }
}