package us.ihmc.robotEnvironmentAwareness.fusion.parameters;

public class PlanarRegionPropagationParameters
{
   private static final double DEFAULT_LOWER_SPARSE_THRESHOLD = 0.01;
   private static final double DEFAULT_UPPER_SPARSE_THRESHOLD = 0.03;
   private static final double DEFAULT_PROXIMITY_THRESHOLD = 0.05;
   private static final double DEFAULT_PLANARITY_THRESHOLD = Math.cos(Math.PI / 180 * 30);

   private static final boolean DEFAULT_ENABLE_EXTENDING = true;
   private static final boolean DEFAULT_UPDATE_EXTENDED_DATA = false;
   private static final double DEFAULT_EXTENDING_DISTANCE_THRESHOLD = 0.01;
   private static final double DEFAULT_EXTENDING_RADIUS_THRESHOLD = 0.03;

   private double sparseLowerThreshold;
   private double sparseUpperThreshold;
   private double proximityThreshold;
   private double planarityThreshold;
   private boolean enableExtending;
   private boolean updateExtendedData;
   private double extendingDistanceThreshold;
   private double extendingRadiusThreshold;

   public PlanarRegionPropagationParameters()
   {
      setDefaultParameters();
   }

   public PlanarRegionPropagationParameters(PlanarRegionPropagationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      sparseLowerThreshold = DEFAULT_LOWER_SPARSE_THRESHOLD;
      sparseUpperThreshold = DEFAULT_UPPER_SPARSE_THRESHOLD;
      proximityThreshold = DEFAULT_PROXIMITY_THRESHOLD;
      planarityThreshold = DEFAULT_PLANARITY_THRESHOLD;

      enableExtending = DEFAULT_ENABLE_EXTENDING;
      updateExtendedData = DEFAULT_UPDATE_EXTENDED_DATA;
      extendingDistanceThreshold = DEFAULT_EXTENDING_DISTANCE_THRESHOLD;
      extendingRadiusThreshold = DEFAULT_EXTENDING_RADIUS_THRESHOLD;
   }

   public void set(PlanarRegionPropagationParameters other)
   {
      sparseLowerThreshold = other.sparseLowerThreshold;
      sparseUpperThreshold = other.sparseUpperThreshold;
      proximityThreshold = other.proximityThreshold;
      planarityThreshold = other.planarityThreshold;

      enableExtending = other.enableExtending;
      updateExtendedData = other.updateExtendedData;
      extendingDistanceThreshold = other.extendingDistanceThreshold;
      extendingRadiusThreshold = other.extendingRadiusThreshold;
   }

   public double getSparseLowerThreshold()
   {
      return sparseLowerThreshold;
   }

   public double getSparseUpperThreshold()
   {
      return sparseUpperThreshold;
   }

   public double getProximityThreshold()
   {
      return proximityThreshold;
   }

   public double getPlanarityThreshold()
   {
      return planarityThreshold;
   }

   public boolean isEnableExtending()
   {
      return enableExtending;
   }

   public boolean isUpdateExtendedData()
   {
      return updateExtendedData;
   }

   public double getExtendingDistanceThreshold()
   {
      return extendingDistanceThreshold;
   }

   public double getExtendingRadiusThreshold()
   {
      return extendingRadiusThreshold;
   }

   public void setSparseLowerThreshold(double sparseLowerThreshold)
   {
      this.sparseLowerThreshold = sparseLowerThreshold;
   }

   public void setSparseUpperThreshold(double sparseUpperThreshold)
   {
      this.sparseUpperThreshold = sparseUpperThreshold;
   }

   public void setProximityThreshold(double proximityThreshold)
   {
      this.proximityThreshold = proximityThreshold;
   }

   public void setPlanarityThreshold(double planarityThreshold)
   {
      this.planarityThreshold = planarityThreshold;
   }

   public void setPlanarityThresholdInDegree(double planarityThresholdInDegree)
   {
      this.planarityThreshold = Math.cos(Math.PI / 180 * planarityThresholdInDegree);
   }

   public void setEnableExtending(boolean enableExtending)
   {
      this.enableExtending = enableExtending;
   }

   public void setUpdateExtendedData(boolean updateExtendedData)
   {
      this.updateExtendedData = updateExtendedData;
   }

   public void setExtendingDistanceThreshold(double extendingDistanceThreshold)
   {
      this.extendingDistanceThreshold = extendingDistanceThreshold;
   }

   public void setExtendingRadiusThreshold(double extendingRadiusThreshold)
   {
      this.extendingRadiusThreshold = extendingRadiusThreshold;
   }
}
