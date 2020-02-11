package us.ihmc.robotEnvironmentAwareness.planarRegion;

public class IhmcSLAMParameters
{
   private double octreeResolution;
   private int numberOfSourcePoints;
   private double maximumDepth;
   private double minimumDepth;
   private double minimumOverlappedRatio;

   private int maximumICPSearchingSize;

   private double minimumInliersRatioOfKeyFrame;

   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   private static final int DEFAULT_NUMBER_OF_SOURCE_POINTS = 300;
   private static final double DEFAULT_MAXIMUM_DEPTH = 0.5;
   private static final double DEFAULT_MINIMUM_DEPTH = 1.5;
   private static final double DEFAULT_MINIMUM_OVERLAPPED_RATIO = 0.4;
   private static final int DEFAULT_MAXIMUM_ICP_SEARCHING_SIZE = 5;
   private static final double DEFAULT_MINIMUM_INLIERS_RATIO_OF_KEY_FRAME = 0.95;

   public IhmcSLAMParameters()
   {
      setDefaultParameters();
   }

   public IhmcSLAMParameters(IhmcSLAMParameters other)
   {
      set(other);
   }
   
   public void set(IhmcSLAMParameters other)
   {
      octreeResolution = other.octreeResolution;
      numberOfSourcePoints = other.numberOfSourcePoints;
      maximumDepth = other.maximumDepth;
      minimumDepth = other.minimumDepth;
      minimumOverlappedRatio = other.minimumOverlappedRatio;
      maximumICPSearchingSize = other.maximumICPSearchingSize;
      minimumInliersRatioOfKeyFrame = other.minimumInliersRatioOfKeyFrame;
   }
   
   public void setDefaultParameters()
   {
      octreeResolution = DEFAULT_OCTREE_RESOLUTION;

      numberOfSourcePoints = DEFAULT_NUMBER_OF_SOURCE_POINTS;
      maximumDepth = DEFAULT_MAXIMUM_DEPTH;
      minimumDepth = DEFAULT_MINIMUM_DEPTH;
      minimumOverlappedRatio = DEFAULT_MINIMUM_OVERLAPPED_RATIO;

      maximumICPSearchingSize = DEFAULT_MAXIMUM_ICP_SEARCHING_SIZE;
      minimumInliersRatioOfKeyFrame = DEFAULT_MINIMUM_INLIERS_RATIO_OF_KEY_FRAME;
   }

   public double getOctreeResolution()
   {
      return octreeResolution;
   }

   public int getNumberOfSourcePoints()
   {
      return numberOfSourcePoints;
   }

   public double getMaximumDepth()
   {
      return maximumDepth;
   }

   public double getMinimumDepth()
   {
      return minimumDepth;
   }

   public double getMinimumOverlappedRatio()
   {
      return minimumOverlappedRatio;
   }

   public int getMaximumICPSearchingSize()
   {
      return maximumICPSearchingSize;
   }

   public double getMinimumInliersRatioOfKeyFrame()
   {
      return minimumInliersRatioOfKeyFrame;
   }

   public void setOctreeResolution(double octreeResolution)
   {
      this.octreeResolution = octreeResolution;
   }

   public void setNumberOfSourcePoints(int numberOfSourcePoints)
   {
      this.numberOfSourcePoints = numberOfSourcePoints;
   }

   public void setMaximumDepth(double maximumDepth)
   {
      this.maximumDepth = maximumDepth;
   }

   public void setMinimumDepth(double minimumDepth)
   {
      this.minimumDepth = minimumDepth;
   }

   public void setMinimumOverlappedRatio(double minimumOverlappedRatio)
   {
      this.minimumOverlappedRatio = minimumOverlappedRatio;
   }

   public void setMaximumICPSearchingSize(int maximumICPSearchingSize)
   {
      this.maximumICPSearchingSize = maximumICPSearchingSize;
   }

   public void setMinimumInliersRatioOfKeyFrame(double minimumInliersRatioOfKeyFrame)
   {
      this.minimumInliersRatioOfKeyFrame = minimumInliersRatioOfKeyFrame;
   }
}
