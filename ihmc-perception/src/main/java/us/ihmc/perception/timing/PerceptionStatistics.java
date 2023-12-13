package us.ihmc.perception.timing;

public class PerceptionStatistics
{
   float timeToComputeHeightMap = 0.0f;
   float timeToComputeRapidRegions = 0.0f;
   float timeToComputeSphericalRapidRegions = 0.0f;
   float timeToComputeMapRegions = 0.0f;

   float averageTimeToComputeHeightMap = 0.0f;
   float averageTimeToComputeRapidRegions = 0.0f;
   float averageTimeToComputeSphericalRapidRegions = 0.0f;
   float averageTimeToComputeMapRegions = 0.0f;

   float timeToDenoiseHeightMap = 0.0f;
   float timeToExtractContactMap = 0.0f;

   public void updateTimeToComputeHeightMap(float timeToComputeHeightMap)
   {
      this.timeToComputeHeightMap = timeToComputeHeightMap;
      averageTimeToComputeHeightMap = (averageTimeToComputeHeightMap + timeToComputeHeightMap) / 2.0f;
   }

   public void updateTimeToComputeRapidRegions(float timeToComputeRapidRegions)
   {
      this.timeToComputeRapidRegions = timeToComputeRapidRegions;
      averageTimeToComputeRapidRegions = (averageTimeToComputeRapidRegions + timeToComputeRapidRegions) / 2.0f;
   }

   public void updateTimeToComputeSphericalRapidRegions(float timeToComputeSphericalRapidRegions)
   {
      this.timeToComputeSphericalRapidRegions = timeToComputeSphericalRapidRegions;
      averageTimeToComputeSphericalRapidRegions = (averageTimeToComputeSphericalRapidRegions + timeToComputeSphericalRapidRegions) / 2.0f;
   }

   public void updateTimeToComputeMapRegions(float timeToComputeMapRegions)
   {
      this.timeToComputeMapRegions = timeToComputeMapRegions;
      averageTimeToComputeMapRegions = (averageTimeToComputeMapRegions + timeToComputeMapRegions) / 2.0f;
   }

   public float getTimeToComputeHeightMap()
   {
      return timeToComputeHeightMap;
   }

   public float getTimeToComputeRapidRegions()
   {
      return timeToComputeRapidRegions;
   }

   public float getTimeToComputeSphericalRapidRegions()
   {
      return timeToComputeSphericalRapidRegions;
   }

   public float getTimeToComputeMapRegions()
   {
      return timeToComputeMapRegions;
   }

   public float getAverageTimeToComputeHeightMap()
   {
      return averageTimeToComputeHeightMap;
   }

   public float getAverageTimeToComputeRapidRegions()
   {
      return averageTimeToComputeRapidRegions;
   }

   public float getAverageTimeToComputeSphericalRapidRegions()
   {
      return averageTimeToComputeSphericalRapidRegions;
   }

   public float getAverageTimeToComputeMapRegions()
   {
      return averageTimeToComputeMapRegions;
   }

   public float getFrequencyHeightMap()
   {
      return 1.0f / averageTimeToComputeHeightMap;
   }

   public float getFrequencyRapidRegions()
   {
      return 1.0f / averageTimeToComputeRapidRegions;
   }

   public float getFrequencySphericalRapidRegions()
   {
      return 1.0f / averageTimeToComputeSphericalRapidRegions;
   }

   public float getFrequencyMapRegions()
   {
      return 1.0f / averageTimeToComputeMapRegions;
   }

   public String toString()
   {
      return "Time Height Map: " + averageTimeToComputeHeightMap + " ms\t" +
             "Time Rapid Regions: " + averageTimeToComputeRapidRegions + " ms\t" +
             "Time Spherical: " + averageTimeToComputeSphericalRapidRegions + " ms\t" +
             "Time Map Regions: " + averageTimeToComputeMapRegions + " ms";
   }
}
