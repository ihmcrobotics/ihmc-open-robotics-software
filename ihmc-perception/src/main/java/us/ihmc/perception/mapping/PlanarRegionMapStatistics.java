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

      totalNumberOfVertices = 0;
      totalSurfaceAreaOnMap = 0.0f;
      for (PlanarRegion region : map.getMapRegions().getPlanarRegionsAsList())
      {
         totalNumberOfVertices += region.getConcaveHullSize();
         totalSurfaceAreaOnMap += region.getArea();
      }

      extractionTime = extractor.getWholeAlgorithmDurationStopwatch().totalElapsed();
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

   public void startMergingTime()
   {
      mergingTime = System.nanoTime() / 1e9;
   }

   public void startRegistrationTime()
   {
      registrationTime = System.nanoTime() / 1e9;
   }

   public void startOptimizationTime()
   {
      optimizationTime = System.nanoTime() / 1e9;
   }

   public void startTotalProcessingTime()
   {
      totalProcessingTime = System.nanoTime() / 1e9;
   }

   public void startExtractionTime()
   {
      extractionTime = System.nanoTime() / 1e9;
   }

   public void stopMergingTime()
   {
      mergingTime = System.nanoTime() / 1e9 - mergingTime;
   }

   public void stopRegistrationTime()
   {
      registrationTime = System.nanoTime() / 1e9 - registrationTime;
   }

   public void stopOptimizationTime()
   {
      optimizationTime = System.nanoTime() / 1e9 - optimizationTime;
   }

   public void stopTotalProcessingTime()
   {
      totalProcessingTime = System.nanoTime() / 1e9 - totalProcessingTime;
   }

   public void stopExtractionTime()
   {
      extractionTime = System.nanoTime() / 1e9 - extractionTime;
   }

   public void reset()
   {
      extractionTime = 0.0f;
      mergingTime = 0.0f;
      registrationTime = 0.0f;
      optimizationTime = 0.0f;
      totalProcessingTime = 0.0f;

      totalNumberOfRegions = 0;
      totalNumberOfVertices = 0;

      totalSurfaceAreaOnMap = 0.0f;
   }

   public double getRegistrationTime()
   {
      return registrationTime;
   }

   public double getOptimizationTime()
   {
      return optimizationTime;
   }

   public double getMergingTime()
   {
      return mergingTime;
   }

   public double getExtractionTime()
   {
      return extractionTime;
   }

   public double getTotalProcessingTime()
   {
      return totalProcessingTime;
   }
}
