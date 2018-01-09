package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.Scanner;

import us.ihmc.jOctoMap.tools.ScannerTools;

public class PlanarRegionSegmentationParameters
{
   private static final double DEFAULT_SEARCH_RADIUS = 0.05;
   private static final double DEFAULT_MAX_DISTANCE_FROM_PLANE = 0.05;
   private static final double DEFAULT_MAX_ANGLE_FROM_PLANE = Math.toRadians(10.0);
   private static final double DEFAULT_MIN_NORMAL_QUALITY = 0.005;
   private static final int DEFAULT_MIN_REGION_SIZE = 50;
   private static final double DEFAULT_MAX_STANDARD_DEVIATION = 0.015;
   private static final double DEFAULT_VOLUMIC_DENSITY = 0.10 * 1.0e6; // cm^3 to m^3

   private double searchRadius;
   private double maxDistanceFromPlane;
   private double maxAngleFromPlane;
   private double minNormalQuality;
   private int minRegionSize;

   private double maxStandardDeviation;
   private double minVolumicDensity;

   public PlanarRegionSegmentationParameters()
   {
      setDefaultParameters();
   }

   public PlanarRegionSegmentationParameters(PlanarRegionSegmentationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      searchRadius = DEFAULT_SEARCH_RADIUS;
      maxDistanceFromPlane = DEFAULT_MAX_DISTANCE_FROM_PLANE;
      maxAngleFromPlane = DEFAULT_MAX_ANGLE_FROM_PLANE;
      minNormalQuality = DEFAULT_MIN_NORMAL_QUALITY;
      minRegionSize = DEFAULT_MIN_REGION_SIZE;

      maxStandardDeviation = DEFAULT_MAX_STANDARD_DEVIATION;
      minVolumicDensity = DEFAULT_VOLUMIC_DENSITY;
   }

   public void set(PlanarRegionSegmentationParameters other)
   {
      searchRadius = other.searchRadius;
      maxDistanceFromPlane = other.maxDistanceFromPlane;
      maxAngleFromPlane = other.maxAngleFromPlane;
      minNormalQuality = other.minNormalQuality;
      minRegionSize = other.minRegionSize;
      maxStandardDeviation = other.maxStandardDeviation;
      minVolumicDensity = other.minVolumicDensity;
   }

   public void setSearchRadius(double searchRadius)
   {
      this.searchRadius = searchRadius;
   }

   public void setMaxDistanceFromPlane(double maxDistanceFromPlane)
   {
      this.maxDistanceFromPlane = maxDistanceFromPlane;
   }

   public void setMaxAngleFromPlane(double maxAngleFromPlane)
   {
      this.maxAngleFromPlane = maxAngleFromPlane;
   }

   public void setMinNormalQuality(double minNormalQuality)
   {
      this.minNormalQuality = minNormalQuality;
   }

   public void setMinRegionSize(int minRegionSize)
   {
      this.minRegionSize = minRegionSize;
   }

   public void setMaxStandardDeviation(double maxStandardDeviation)
   {
      this.maxStandardDeviation = maxStandardDeviation;
   }

   public void setMinVolumicDensity(double minVolumicDensity)
   {
      this.minVolumicDensity = minVolumicDensity;
   }

   public double getSearchRadius()
   {
      return searchRadius;
   }

   public double getMaxDistanceFromPlane()
   {
      return maxDistanceFromPlane;
   }

   public double getMaxAngleFromPlane()
   {
      return maxAngleFromPlane;
   }

   public double getMinNormalQuality()
   {
      return minNormalQuality;
   }

   public int getMinRegionSize()
   {
      return minRegionSize;
   }

   public double getMaxStandardDeviation()
   {
      return maxStandardDeviation;
   }

   public double getMinVolumicDensity()
   {
      return minVolumicDensity;
   }

   @Override
   public String toString()
   {
      return "search radius: " + searchRadius + ", max distance from plane: " + maxDistanceFromPlane
            + ", maxAngleFromPlane: " + maxAngleFromPlane + ", minNormalQuality: " + minNormalQuality + ", min region size: " + minRegionSize
            + ", max standard deviation: " + maxStandardDeviation + ", min volumic density: " + minVolumicDensity;
   }

   public static PlanarRegionSegmentationParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      PlanarRegionSegmentationParameters parameters = new PlanarRegionSegmentationParameters();
      parameters.setSearchRadius(ScannerTools.readNextDouble(scanner, parameters.getSearchRadius()));
      parameters.setMaxDistanceFromPlane(ScannerTools.readNextDouble(scanner, parameters.getMaxDistanceFromPlane()));
      parameters.setMaxAngleFromPlane(ScannerTools.readNextDouble(scanner, parameters.getMaxAngleFromPlane()));
      parameters.setMinNormalQuality(ScannerTools.readNextDouble(scanner, parameters.getMinNormalQuality()));
      parameters.setMinRegionSize(ScannerTools.readNextInt(scanner, parameters.getMinRegionSize()));
      parameters.setMaxStandardDeviation(ScannerTools.readNextDouble(scanner, parameters.getMaxStandardDeviation()));
      parameters.setMinVolumicDensity(ScannerTools.readNextDouble(scanner, parameters.getMinVolumicDensity()));
      scanner.close();
      return parameters;
   }
}
