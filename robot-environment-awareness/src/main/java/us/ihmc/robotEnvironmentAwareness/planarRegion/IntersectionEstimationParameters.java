package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.Scanner;

public class IntersectionEstimationParameters
{
   private double maxDistanceToRegion;
   private int minRegionSize;
   private double minIntersectionLength;
   private double minRegionAngleDifference;
   private boolean addIntersectionsToRegions;

   public IntersectionEstimationParameters()
   {
      setDefaultParameters();
   }

   public IntersectionEstimationParameters(IntersectionEstimationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      maxDistanceToRegion = 0.10;
      minRegionSize = 50;
      minIntersectionLength = 0.06;
      minRegionAngleDifference = Math.toRadians(15.0);
      addIntersectionsToRegions = true;
   }

   public void set(IntersectionEstimationParameters other)
   {
      maxDistanceToRegion = other.maxDistanceToRegion;
      minRegionSize = other.minRegionSize;
      minIntersectionLength = other.minIntersectionLength;
      minRegionAngleDifference = other.minRegionAngleDifference;
      addIntersectionsToRegions = other.addIntersectionsToRegions;
   }

   public void setMaxDistanceToRegion(double maxDistanceToRegion)
   {
      this.maxDistanceToRegion = maxDistanceToRegion;
   }

   public void setMinRegionSize(int minRegionSize)
   {
      this.minRegionSize = minRegionSize;
   }

   public void setMinIntersectionLength(double minIntersectionLength)
   {
      this.minIntersectionLength = minIntersectionLength;
   }

   public void setMinRegionAngleDifference(double minRegionAngleDifference)
   {
      this.minRegionAngleDifference = minRegionAngleDifference;
   }

   public void setAddIntersectionsToRegions(boolean addIntersectionsToRegions)
   {
      this.addIntersectionsToRegions = addIntersectionsToRegions;
   }

   public double getMaxDistanceToRegion()
   {
      return maxDistanceToRegion;
   }

   public int getMinRegionSize()
   {
      return minRegionSize;
   }

   public double getMinIntersectionLength()
   {
      return minIntersectionLength;
   }

   public double getMinRegionAngleDifference()
   {
      return minRegionAngleDifference;
   }

   public boolean isAddIntersectionsToRegions()
   {
      return addIntersectionsToRegions;
   }

   @Override
   public String toString()
   {
      return "maxDistanceToRegion: " + maxDistanceToRegion + ", minRegionSize: " + minRegionSize + ", minIntersectionLength: " + minIntersectionLength
            + ", minRegionAngleDifference: " + minRegionAngleDifference + ", addIntersectionsToRegions: " + addIntersectionsToRegions;
   }

   public static IntersectionEstimationParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      while (!scanner.hasNextDouble())
         scanner.next();
      double maxDistanceToRegion = scanner.nextDouble();
      while (!scanner.hasNextInt())
         scanner.next();
      int minRegionSize = scanner.nextInt();
      while (!scanner.hasNextDouble())
         scanner.next();
      double minIntersectionLength = scanner.nextDouble();
      while (!scanner.hasNextDouble())
         scanner.next();
      double minRegionAngleDifference = scanner.nextDouble();
      while (!scanner.hasNextBoolean())
         scanner.next();
      boolean addIntersectionsToRegions = scanner.nextBoolean();
      scanner.close();

      IntersectionEstimationParameters parameters = new IntersectionEstimationParameters();
      parameters.setMaxDistanceToRegion(maxDistanceToRegion);
      parameters.setMinRegionSize(minRegionSize);
      parameters.setMinIntersectionLength(minIntersectionLength);
      parameters.setMinRegionAngleDifference(minRegionAngleDifference);
      parameters.setAddIntersectionsToRegions(addIntersectionsToRegions);
      return parameters;
   }
}
