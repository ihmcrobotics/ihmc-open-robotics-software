package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.log.LogTools;
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
   private LidarImageFusionRawData rawData;
   private final AtomicReference<Integer> seedLabel;
   private final Random random = new Random(0612L);

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

      messager.registerTopicListener(LidarImageFusionAPI.ClearObjects, (content) -> detectedObjectViewer.clear());
      messager.registerTopicListener(LidarImageFusionAPI.LoadData, (content) -> loadData());
      messager.registerTopicListener(LidarImageFusionAPI.ClearViz, (content) -> clearViz());
      messager.registerTopicListener(LidarImageFusionAPI.VisualizeAll, (content) -> visualizeAll());
      messager.registerTopicListener(LidarImageFusionAPI.Propagate, (content) -> propagate());
      messager.registerTopicListener(LidarImageFusionAPI.PlanarRegion, (content) -> planarRegion());
      messager.registerTopicListener(LidarImageFusionAPI.EndToEnd, (content) -> endToEnd());
      seedLabel = messager.createInput(LidarImageFusionAPI.Seed);

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

   private LidarImageFusionDataFeatureUpdater updater;
   private void loadData()
   {
      dataLoader.loadLidarImageFusionRawData("dataOne", pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
      rawData = dataLoader.getRawData("dataOne");
      rawData.initializeSegments();
      
      updater = new LidarImageFusionDataFeatureUpdater(rawData);
   }

   private void clearViz()
   {
      root.getChildren().clear();
   }

   private void visualizeAll()
   {
      int numberOfLabels = rawData.getNumberOfLabels();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();

      for (int i = 0; i < numberOfLabels; i++)
      {
         FusionDataSegment fusionDataSegment = rawData.getFusionDataSegment(i);
         Point3D labelCenter = fusionDataSegment.getCenter();
         Vector3D labelNormal = fusionDataSegment.getNormal();
         Point3D labelNormalEnd = new Point3D(labelNormal);
         labelNormalEnd.scaleAdd(0.05, labelCenter);

         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.01), labelCenter, Color.PINK);
         meshBuilder.addLine(labelCenter, labelNormalEnd, 0.01, Color.RED);
         for(Point3D point:rawData.getFusionDataSegment(i).getPoints())
         {
            //meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.01), point, Color.GREEN);
         }
      }
      MeshView newScanMeshView = new MeshView(meshBuilder.generateMesh());
      newScanMeshView.setMaterial(meshBuilder.generateMaterial());
      root.getChildren().add(newScanMeshView);
   }

   private int manualPropagateIndex = 0;
   private void propagate()
   {
      double randomB = random.nextDouble();
      double randomG = random.nextDouble();
      double randomR = random.nextDouble();
      LogTools.info(" " + randomR + " " + randomG + " " + randomB);
      Color color = new Color(randomR, randomG, randomB, 1.0);

      long startTime = System.nanoTime();
      updater.addSegmentNodeData(seedLabel.get(), manualPropagateIndex);
      LogTools.info("propagate " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();
      List<Point3D> pointsOnSegment = updater.getPointsOnSegment(manualPropagateIndex);
      for (Point3D point : pointsOnSegment)
      {
         meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.01), point, color);
      }
      
      Point3D labelCenter = updater.getSegmentCenter(manualPropagateIndex);
      Vector3D labelNormal = updater.getSegmentNormal(manualPropagateIndex);
      Point3D labelNormalEnd = new Point3D(labelNormal);
      labelNormalEnd.scaleAdd(0.05, labelCenter);

      meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.01), labelCenter, color);
      meshBuilder.addLine(labelCenter, labelNormalEnd, 0.01, color);
      
      MeshView newScanMeshView = new MeshView(meshBuilder.generateMesh());
      newScanMeshView.setMaterial(meshBuilder.generateMaterial());

      root.getChildren().add(newScanMeshView);
      manualPropagateIndex++;
   }

   private void planarRegion()
   {

   }

   private void endToEnd()
   {
      updater.initialize();
      long startTime = System.nanoTime();
      int numberOfIterate = 50;
      for (int i = 0; i < numberOfIterate; i++)
      {
         if (!updater.iterateSegmenataionPropagation(i))
         {
            LogTools.info("iterative is terminated " + i);
            break;
         }

         double randomB = random.nextDouble();
         double randomG = random.nextDouble();
         double randomR = random.nextDouble();
         Color color = new Color(randomR, randomG, randomB, 1.0);

         JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();
         List<Point3D> pointsOnSegment = updater.getPointsOnSegment(i);
         for (Point3D point : pointsOnSegment)
         {
            meshBuilder.addMesh(MeshDataGenerator.Tetrahedron(0.01), point, color);
         }
         MeshView newScanMeshView = new MeshView(meshBuilder.generateMesh());
         newScanMeshView.setMaterial(meshBuilder.generateMaterial());

         root.getChildren().add(newScanMeshView);
      }
      LogTools.info("endToEnd " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
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