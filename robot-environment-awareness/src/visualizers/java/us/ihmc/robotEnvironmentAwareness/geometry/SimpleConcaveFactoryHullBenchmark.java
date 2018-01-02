package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.File;
import java.io.IOException;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationRawDataImporter;

/**
 * Simple infinite loop on the {@link SimpleConcaveHullFactory} for convenience for profiling the algorithm.
 * @author Sylvain Bertrand
 *
 */
public class SimpleConcaveFactoryHullBenchmark
{
   public static void main(String[] args) throws IOException
   {
      PlanarRegionSegmentationRawDataImporter dataImporter = new PlanarRegionSegmentationRawDataImporter(new File("../../Data/20161210_185643_PlanarRegionSegmentation_Atlas_CB"));
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationRawData> regionsRawData = dataImporter.getPlanarRegionSegmentationRawData();
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();

      while (true)
      {
         for (PlanarRegionSegmentationRawData rawData : regionsRawData)
         {
            List<Point2D> pointsInPlane = rawData.getPointCloudInPlane();
            SimpleConcaveHullFactory.createConcaveHull(pointsInPlane, parameters);
         }
      }
   }
}
