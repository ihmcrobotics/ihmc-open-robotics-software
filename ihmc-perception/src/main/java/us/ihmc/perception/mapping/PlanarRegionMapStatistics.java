package us.ihmc.perception.mapping;

import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionMapStatistics
{
   private double extractionTime = 0.0f;
   private double mergingTime = 0.0f;
   private double registrationTime = 0.0f;
   private double optimizationTime = 0.0f;
   private double totalProcessingTime = 0.0f;

   private int totalNumberOfRegions = 0;
   private int totalNumberOfVertices = 0;

   private double totalSurfaceAreaOnMap = 0.0f;

   public PlanarRegionMapStatistics()
   {
   }

   public void computeStatistics(PlanarRegionMap map, RapidPlanarRegionsExtractor extractor)
   {
      totalNumberOfRegions = map.getMapRegions().getNumberOfPlanarRegions();

      for (PlanarRegion region : map.getMapRegions().getPlanarRegionsAsList())
      {
         totalNumberOfVertices += region.getConcaveHullSize();
         totalSurfaceAreaOnMap += region.getArea();
      }

      extractionTime = extractor.getWholeAlgorithmDurationStopwatch().totalElapsed();
      mergingTime = map.getRegionMergingStopwatch().totalElapsed();
      registrationTime = map.getQuaternionAveragingStopwatch().totalElapsed();
      optimizationTime = map.getFactorGraphStopwatch().totalElapsed();
      totalProcessingTime = extractionTime + mergingTime + registrationTime + optimizationTime;
   }

   public String getHeader()
   {
      String result = "Map Statistics: {";

      result += "Regions, ";
      result += "Vertices, ";
      result += "Area, ";
      result += "Extraction, ";
      result += "Merging, ";
      result += "Registration, ";
      result += "Optimization, ";

      return result;
   }

   public String toCSV()
   {
      return String.format("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d",
                           extractionTime, mergingTime,
                           registrationTime,
                           optimizationTime,
                           totalProcessingTime,
                           totalSurfaceAreaOnMap,
                           totalNumberOfRegions,
                           totalNumberOfVertices);
   }

   public String toString()
   {
      return String.format(
            "Extraction Time: %.3f, Merging Time: %.3f, Registration Time: %.3f, Optimization Time: %.3f, Total Time: %.3f, Area: %.3f, Regions: %d, Vertices: %d",
            extractionTime,
            mergingTime,
            registrationTime,
            optimizationTime,
            totalProcessingTime,
            totalSurfaceAreaOnMap,
            totalNumberOfRegions,
            totalNumberOfVertices);
   }
}
