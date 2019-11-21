package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.Collections;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class EnvironmentMappingViewer extends Application
{
   private static final boolean SHOW_PLANAR_REGIONS = false;
   private static final boolean SHOW_STEREO_POINT_CLOUD = true;

   private static final String PLANAR_REGIONS_FILE_NAME = "PlanarRegion";
   private static final String POINT_CLOUD_FILE_NAME = "PointCloud";

   private static final boolean USE_FRAME_FILTER = true;
   private static final double REFERENCE_OCTREE_RESOLUTION = 0.02;

   private StereoVisionPointCloudMessage testFrame;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      StereoVisionPointCloudGraphic stereoVisionPointCloudGraphic = new StereoVisionPointCloudGraphic();

      File dataFolder = PlanarRegionDataImporter.chooseFile(primaryStage);
      File[] listOfFiles = dataFolder.listFiles();

      if (SHOW_PLANAR_REGIONS)
      {
         File planarRegionsFile = null;
         for (File file : listOfFiles)
         {
            String fileName = file.getName();

            if (fileName.contains(PLANAR_REGIONS_FILE_NAME))
            {
               planarRegionsFile = file;
               break;
            }
         }

         if (planarRegionsFile == null)
         {
            System.out.println("No planar regions file.");
         }
         else
         {
            regionsGraphic.generateMeshes(PlanarRegionFileTools.importPlanarRegionData(planarRegionsFile));
            regionsGraphic.update();
            view3dFactory.addNodeToView(regionsGraphic);
            System.out.println("Planar regions are rendered.");
         }
      }

      if (SHOW_STEREO_POINT_CLOUD)
      {
         File pointCloudFile = null;

         for (File file : listOfFiles)
         {
            String fileName = file.getName();

            if (fileName.contains(POINT_CLOUD_FILE_NAME))
            {
               pointCloudFile = file;
               break;
            }
         }

         if (pointCloudFile == null)
         {
            System.out.println("No point cloud file.");
         }
         else
         {
            List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
            System.out.println("Point cloud messages (" + messagesFromFile.size() + ")");
            stereoVisionPointCloudGraphic.generateMeshes(messagesFromFile);
            stereoVisionPointCloudGraphic.update();
            view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);
            System.out.println("are rendered.");

            testFrame = messagesFromFile.get(0);
         }
      }

      regionsGraphic.generateMeshes(computePlanarRegion(testFrame));
      regionsGraphic.update();
      view3dFactory.addNodeToView(regionsGraphic);

      primaryStage.setTitle(dataFolder.getPath());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }

   private PlanarRegionsList computePlanarRegion(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      NormalOcTree node = createOctreeNode(stereoVisionPointCloudMessage);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();
      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(stereoVisionPointCloudMessage.getSensorPosition());

      segmentationCalculator.compute(node.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

      PlanarRegionsList planarRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      return planarRegionsList;
   }

   private NormalOcTree createOctreeNode(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = stereoVisionPointCloudMessage.getColors().size();

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(toScan(stereoVisionPointCloudMessage.getPointCloud(), stereoVisionPointCloudMessage.getSensorPosition()));

      Pose3D sensorPose = new Pose3D();
      sensorPose.setPosition(stereoVisionPointCloudMessage.getSensorPosition());
      sensorPose.setOrientation(stereoVisionPointCloudMessage.getSensorOrientation());

      NormalOcTree referenceOctree = new NormalOcTree(REFERENCE_OCTREE_RESOLUTION);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();

      return referenceOctree;
   }

   private static Scan toScan(Float data, Point3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();

      int bufferIndex = 0;

      while (bufferIndex < data.size())
      {
         float x = data.getQuick(bufferIndex++);
         float y = data.getQuick(bufferIndex++);
         float z = data.getQuick(bufferIndex++);
         pointCloud.add(x, y, z);
      }
      return new Scan(sensorPosition, pointCloud);
   }
}
