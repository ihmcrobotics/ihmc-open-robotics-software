package us.ihmc.robotEnvironmentAwareness.geometry;

import org.junit.jupiter.api.Test;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
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
   @Test
   public void rawDataConstructionTest()
   {
      long startTime = System.nanoTime();
      dataLoader.loadLidarImageFusionRawData("dataOne", pointCloudDataFileName, labeledImageDataFileName, imageWidth, imageHeight, intrinsicParameters);
      LogTools.info("The construction tooks time " + Conversions.nanosecondsToMilliseconds(System.nanoTime() - startTime) + " ms");
   }

   @Test
   public void dataInitializeTest()
   {

   }

   @Test
   public void segmentVisualizationTest()
   {

   }

   @Test
   public void segmentPropagationTest()
   {

   }

   @Test
   public void segmentationEndToEndTest()
   {

   }
}
