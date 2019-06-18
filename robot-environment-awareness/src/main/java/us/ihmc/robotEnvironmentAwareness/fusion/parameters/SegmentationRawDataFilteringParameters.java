package us.ihmc.robotEnvironmentAwareness.fusion.parameters;

public class SegmentationRawDataFilteringParameters
{
   private static final double DEFAULT_MINIMUM_SPARSE_THRESHOLD = 0.03;
   private static final double DEFAULT_MAXIMUM_SPARSE_PROPOTIONAL_RATIO = 2.0;

   private static final boolean DEFAULT_ENABLE_FILTER_FLYING_POINT = true;
   private static final double DEFAULT_FLYING_POINT_THRESHOLD = 0.03;
   private static final int DEFAULT_MINIMUM_NUMBER_OF_FLYING_POINT_NEIGHTBORS = 5;

   private static final boolean DEFAULT_ENABLE_FILTER_CENTRALITY = true;
   private static final double DEFAULT_CENTRALITY_RADIUS = 0.03;
   private static final double DEFAULT_CENTRALITY_THRESHOLD = 0.5;

   private static final boolean DEFAULT_ENABLE_FILTER_ELLIPTICITY = true;
   private static final double DEFAULT_ELLIPTICITY_MINIMUM_LENGTH = 0.02;
   private static final double DEFAULT_ELLIPTICITY_THRESHOLD = 2.0;

   private double minimumSparseThreshold;
   private double maximumSparsePropotionalRatio;

   private boolean enableFilterFlyingPoint;
   private double flyingPointThreshold;
   private int minimumNumberOfFlyingPointNeighbors;

   private boolean enableFilterCentrality;
   private double centralityRadius;
   private double centralityThreshold;

   private boolean enableFilterEllipticity;
   private double ellipticityMinimumLength;
   private double ellipticityThreshold;

   public SegmentationRawDataFilteringParameters()
   {
      setDefaultParameters();
   }

   public SegmentationRawDataFilteringParameters(SegmentationRawDataFilteringParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      minimumSparseThreshold = DEFAULT_MINIMUM_SPARSE_THRESHOLD;
      maximumSparsePropotionalRatio = DEFAULT_MAXIMUM_SPARSE_PROPOTIONAL_RATIO;

      enableFilterFlyingPoint = DEFAULT_ENABLE_FILTER_FLYING_POINT;
      flyingPointThreshold = DEFAULT_FLYING_POINT_THRESHOLD;
      minimumNumberOfFlyingPointNeighbors = DEFAULT_MINIMUM_NUMBER_OF_FLYING_POINT_NEIGHTBORS;

      enableFilterCentrality = DEFAULT_ENABLE_FILTER_CENTRALITY;
      centralityRadius = DEFAULT_CENTRALITY_RADIUS;
      centralityThreshold = DEFAULT_CENTRALITY_THRESHOLD;

      enableFilterEllipticity = DEFAULT_ENABLE_FILTER_ELLIPTICITY;
      ellipticityMinimumLength = DEFAULT_ELLIPTICITY_MINIMUM_LENGTH;
      ellipticityThreshold = DEFAULT_ELLIPTICITY_THRESHOLD;
   }

   public void set(SegmentationRawDataFilteringParameters other)
   {
      minimumSparseThreshold = other.minimumSparseThreshold;
      maximumSparsePropotionalRatio = other.maximumSparsePropotionalRatio;

      enableFilterFlyingPoint = other.enableFilterFlyingPoint;
      flyingPointThreshold = other.flyingPointThreshold;
      minimumNumberOfFlyingPointNeighbors = other.minimumNumberOfFlyingPointNeighbors;

      enableFilterCentrality = other.enableFilterCentrality;
      centralityRadius = other.centralityRadius;
      centralityThreshold = other.centralityThreshold;

      enableFilterEllipticity = other.enableFilterEllipticity;
      ellipticityMinimumLength = other.ellipticityMinimumLength;
      ellipticityThreshold = other.ellipticityThreshold;
   }

   public double getMinimumSparseThreshold()
   {
      return minimumSparseThreshold;
   }

   public double getMaximumSparsePropotionalRatio()
   {
      return maximumSparsePropotionalRatio;
   }

   public boolean isEnableFilterFlyingPoint()
   {
      return enableFilterFlyingPoint;
   }

   public double getFlyingPointThreshold()
   {
      return flyingPointThreshold;
   }

   public int getMinimumNumberOfFlyingPointNeighbors()
   {
      return minimumNumberOfFlyingPointNeighbors;
   }

   public boolean isEnableFilterCentrality()
   {
      return enableFilterCentrality;
   }

   public double getCentralityRadius()
   {
      return centralityRadius;
   }

   public double getCentralityThreshold()
   {
      return centralityThreshold;
   }

   public boolean isEnableFilterEllipticity()
   {
      return enableFilterEllipticity;
   }

   public double getEllipticityMinimumLength()
   {
      return ellipticityMinimumLength;
   }

   public double getEllipticityThreshold()
   {
      return ellipticityThreshold;
   }

   public void setMinimumSparseThreshold(double minimumSparseThreshold)
   {
      this.minimumSparseThreshold = minimumSparseThreshold;
   }

   public void setMaximumSparsePropotionalRatio(double maximumSparsePropotionalRatio)
   {
      this.maximumSparsePropotionalRatio = maximumSparsePropotionalRatio;
   }

   public void setEnableFilterFlyingPoint(boolean enableFilterFlyingPoint)
   {
      this.enableFilterFlyingPoint = enableFilterFlyingPoint;
   }

   public void setFlyingPointThreshold(double flyingPointThreshold)
   {
      this.flyingPointThreshold = flyingPointThreshold;
   }

   public void setMinimumNumberOfFlyingPointNeighbors(int minimumNumberOfFlyingPointNeighbors)
   {
      this.minimumNumberOfFlyingPointNeighbors = minimumNumberOfFlyingPointNeighbors;
   }

   public void setEnableFilterCentrality(boolean enableFilterCentrality)
   {
      this.enableFilterCentrality = enableFilterCentrality;
   }

   public void setCentralityRadius(double centralityRadius)
   {
      this.centralityRadius = centralityRadius;
   }

   public void setCentralityThreshold(double centralityThreshold)
   {
      this.centralityThreshold = centralityThreshold;
   }

   public void setEnableFilterEllipticity(boolean enableFilterEllipticity)
   {
      this.enableFilterEllipticity = enableFilterEllipticity;
   }

   public void setEllipticityMinimumLength(double ellipticityMinimumLength)
   {
      this.ellipticityMinimumLength = ellipticityMinimumLength;
   }

   public void setEllipticityThreshold(double ellipticityThreshold)
   {
      this.ellipticityThreshold = ellipticityThreshold;
   }
}
