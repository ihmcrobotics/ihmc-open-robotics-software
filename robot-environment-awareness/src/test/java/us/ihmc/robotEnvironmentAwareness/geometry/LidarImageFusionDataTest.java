package us.ihmc.robotEnvironmentAwareness.geometry;

import org.junit.jupiter.api.Test;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionDataFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionRawData;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionRawDataLoader;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;

public class LidarImageFusionDataTest
{
   private static final String pointCloudDataFileName = "C:\\Users\\inhol\\Desktop\\SavedData\\stereovisionpointcloud.txt";
   private static final String labeledImageDataFileName = "C:\\Users\\inhol\\Desktop\\SavedData\\labeledimage.txt";
   private static final int imageWidth = 1024;
   private static final int imageHeight = 544;

   private final IntrinsicParameters intrinsicParameters = PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters;
   private final LidarImageFusionRawDataLoader dataLoader = new LidarImageFusionRawDataLoader();

   //TODO: define some parameters here as private static final values.
   //   @Test
   //   public void rawDataConstructionTest()
   //   {
   //      dataLoader.loadLidarImageFusionRawData("dataOne", pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
   //   }
   //
   @Test
   public void dataInitializeTest()
   {
      String dataName = "dataOne";
      dataLoader.loadLidarImageFusionRawData(dataName, pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
      LidarImageFusionRawData rawData = dataLoader.getRawData(dataName);
      long startTime = System.nanoTime();
      rawData.initializeSegments();
      LogTools.info("initializeSegments time is " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
   }

   @Test
   public void segmentSinglePropagationTest()
   {
      String dataName = "dataOne";
      dataLoader.loadLidarImageFusionRawData(dataName, pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
      LidarImageFusionRawData rawData = dataLoader.getRawData(dataName);
      rawData.initializeSegments();
      LidarImageFusionDataFeatureUpdater updater = new LidarImageFusionDataFeatureUpdater(rawData);

//      long startTime = System.nanoTime();
//      updater.iterateSegmenataionPropagation(0);
//      LogTools.info("propagating time is " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
//      
//      updater.createSegmentNodeData(403, 1);
//      
//      updater.createSegmentNodeData(253, 2);
//      
//      updater.createSegmentNodeData(130, 3);
      updater.createSegmentNodeData(407, 1);
   }

   @Test
   public void segmentationEndToEndTest()
   {

   }
}
