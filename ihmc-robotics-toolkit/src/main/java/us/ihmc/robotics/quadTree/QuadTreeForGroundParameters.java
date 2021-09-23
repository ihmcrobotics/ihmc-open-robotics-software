package us.ihmc.robotics.quadTree;

public class QuadTreeForGroundParameters
{
   private double xyResolution;
   private double heightThreshold;
   private double maxMultiLevelZChangeToFilterNoise;
   private int maxSameHeightPointsPerNode;
   private double maxAllowableXYDistanceForAPointToBeConsideredClose;
   private int maximumNumberOfPoints;


   public QuadTreeForGroundParameters(double resolution, double heightThreshold, double maxMultiLevelZChangeToFilterNoise, int maxSameHeightPointsPerNode,
           double maxAllowableXYDistanceForAPointToBeConsideredClose, int maximumNumberOfPoints)
   {
      this.xyResolution = resolution;
      this.heightThreshold = heightThreshold;
      this.maxMultiLevelZChangeToFilterNoise = maxMultiLevelZChangeToFilterNoise;
      this.maxSameHeightPointsPerNode = maxSameHeightPointsPerNode;
      this.maxAllowableXYDistanceForAPointToBeConsideredClose = maxAllowableXYDistanceForAPointToBeConsideredClose;
      this.maximumNumberOfPoints = maximumNumberOfPoints;
   }

   public double getXYResolution()
   {
      return xyResolution;
   }

   public void setXyResolution(double xyResolution)
   {
      this.xyResolution = xyResolution;
   }

   public double getHeightThreshold()
   {
      return heightThreshold;
   }

   public void setHeightThreshold(double heightThreshold)
   {
      this.heightThreshold = heightThreshold;
   }

   public double getMaxMultiLevelZChangeToFilterNoise()
   {
      return maxMultiLevelZChangeToFilterNoise;
   }

   public void setMaxMultiLevelZChangeToFilterNoise(double maxMultiLevelZChangeToFilterNoise)
   {
      this.maxMultiLevelZChangeToFilterNoise = maxMultiLevelZChangeToFilterNoise;
   }

   public int getMaxSameHeightPointsPerNode()
   {
      return maxSameHeightPointsPerNode;
   }

   public void setMaxSameHeightPointsPerNode(int maxSameHeightPointsPerNode)
   {
      this.maxSameHeightPointsPerNode = maxSameHeightPointsPerNode;
   }

   public double getMaxAllowableXYDistanceForAPointToBeConsideredClose()
   {
      return maxAllowableXYDistanceForAPointToBeConsideredClose;
   }

   public void setMaxAllowableXYDistanceForAPointToBeConsideredClose(double maxAllowableXYDistanceForAPointToBeConsideredClose)
   {
      this.maxAllowableXYDistanceForAPointToBeConsideredClose = maxAllowableXYDistanceForAPointToBeConsideredClose;
   }

   public int getMaximumNumberOfPoints()
   {
      return maximumNumberOfPoints;
   }

   public void setMaximumNumberOfPoints(int maximumNumberOfPoints)
   {
      this.maximumNumberOfPoints = maximumNumberOfPoints;
   }
}
