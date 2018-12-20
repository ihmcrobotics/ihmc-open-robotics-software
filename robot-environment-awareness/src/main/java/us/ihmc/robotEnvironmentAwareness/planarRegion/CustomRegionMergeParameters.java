package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.jOctoMap.tools.ScannerTools;

import java.util.Scanner;

public class CustomRegionMergeParameters
{
   private static final double DEFAULT_SEARCH_RADIUS = 0.22;
   private static final double DEFAULT_MAX_DISTANCE_FROM_PLANE = 0.05;
   private static final double DEFAULT_MAX_ANGLE_FROM_PLANE = Math.toRadians(10.0);

   private double searchRadius;
   private double maxDistanceFromPlane;
   private double maxAngleFromPlane;

   public CustomRegionMergeParameters()
   {
      setDefaultParameters();
   }

   public CustomRegionMergeParameters(CustomRegionMergeParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      searchRadius = DEFAULT_SEARCH_RADIUS;
      maxDistanceFromPlane = DEFAULT_MAX_DISTANCE_FROM_PLANE;
      maxAngleFromPlane = DEFAULT_MAX_ANGLE_FROM_PLANE;
   }

   public void set(CustomRegionMergeParameters other)
   {
      searchRadius = other.searchRadius;
      maxDistanceFromPlane = other.maxDistanceFromPlane;
      maxAngleFromPlane = other.maxAngleFromPlane;
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

   @Override
   public String toString()
   {
      return "search radius: " + searchRadius + ", max distance from plane: " + maxDistanceFromPlane
            + ", maxAngleFromPlane: " + maxAngleFromPlane;
   }

   public static CustomRegionMergeParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      CustomRegionMergeParameters parameters = new CustomRegionMergeParameters();
      parameters.setSearchRadius(ScannerTools.readNextDouble(scanner, parameters.getSearchRadius()));
      parameters.setMaxDistanceFromPlane(ScannerTools.readNextDouble(scanner, parameters.getMaxDistanceFromPlane()));
      parameters.setMaxAngleFromPlane(ScannerTools.readNextDouble(scanner, parameters.getMaxAngleFromPlane()));
      scanner.close();
      return parameters;
   }
}
