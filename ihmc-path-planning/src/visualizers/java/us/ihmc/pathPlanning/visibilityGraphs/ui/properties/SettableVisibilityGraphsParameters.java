package us.ihmc.pathPlanning.visibilityGraphs.ui.properties;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;

public class SettableVisibilityGraphsParameters implements VisibilityGraphsParameters
{
   private double maxInterRegionConnectionLength;
   private double normalZThresholdForAccessibleRegions;
   private double preferredExtrusionDistance;
   private double extrusionDistance;
   private double extrusionDistanceIfNotTooHighToStep;
   private double tooHighToStepDistance;
   private double clusterResolution;
   private double explorationDistanceFromStartGoal;
   private double planarRegionMinArea;
   private int planarRegionMinSize;
   private double regionOrthogonalAngle;
   private double searchHostRegionEpsilon;

   public SettableVisibilityGraphsParameters(VisibilityGraphsParameters visibilityGraphsParameters)
   {
      set(visibilityGraphsParameters);
   }

   public void set(VisibilityGraphsParameters visibilityGraphsParameters)
   {
      setMaxInterRegionConnectionLength(visibilityGraphsParameters.getMaxInterRegionConnectionLength());
      setNormalZThresholdForAccessibleRegions(visibilityGraphsParameters.getNormalZThresholdForAccessibleRegions());
      setPreferredExtrusionDistance(visibilityGraphsParameters.getPreferredObstacleExtrusionDistance());
      setExtrusionDistance(visibilityGraphsParameters.getObstacleExtrusionDistance());
      setExtrusionDistanceIfNotTooHighToStep(visibilityGraphsParameters.getObstacleExtrusionDistanceIfNotTooHighToStep());
      setTooHighToStepDistance(visibilityGraphsParameters.getTooHighToStepDistance());
      setClusterResolution(visibilityGraphsParameters.getClusterResolution());
      setExplorationDistanceFromStartGoal(visibilityGraphsParameters.getExplorationDistanceFromStartGoal());
      setPlanarRegionMinArea(visibilityGraphsParameters.getPlanarRegionMinArea());
      setPlanarRegionMinSize(visibilityGraphsParameters.getPlanarRegionMinSize());
      setRegionOrthogonalAngle(visibilityGraphsParameters.getRegionOrthogonalAngle());
      setSearchHostRegionEpsilon(visibilityGraphsParameters.getSearchHostRegionEpsilon());
   }

   public void setMaxInterRegionConnectionLength(double maxInterRegionConnectionLength)
   {
      this.maxInterRegionConnectionLength = maxInterRegionConnectionLength;
   }

   public void setNormalZThresholdForAccessibleRegions(double normalZThresholdForAccessibleRegions)
   {
      this.normalZThresholdForAccessibleRegions = normalZThresholdForAccessibleRegions;
   }

   public void setPreferredExtrusionDistance(double preferredExtrusionDistance)
   {
      this.preferredExtrusionDistance = preferredExtrusionDistance;
   }

   public void setExtrusionDistance(double extrusionDistance)
   {
      this.extrusionDistance = extrusionDistance;
   }

   public void setExtrusionDistanceIfNotTooHighToStep(double extrusionDistanceIfNotTooHighToStep)
   {
      this.extrusionDistanceIfNotTooHighToStep = extrusionDistanceIfNotTooHighToStep;
   }

   public void setTooHighToStepDistance(double tooHighToStepDistance)
   {
      this.tooHighToStepDistance = tooHighToStepDistance;
   }

   public void setClusterResolution(double clusterResolution)
   {
      this.clusterResolution = clusterResolution;
   }

   public void setExplorationDistanceFromStartGoal(double explorationDistanceFromStartGoal)
   {
      this.explorationDistanceFromStartGoal = explorationDistanceFromStartGoal;
   }

   public void setPlanarRegionMinArea(double planarRegionMinArea)
   {
      this.planarRegionMinArea = planarRegionMinArea;
   }

   public void setPlanarRegionMinSize(int planarRegionMinSize)
   {
      this.planarRegionMinSize = planarRegionMinSize;
   }

   public void setRegionOrthogonalAngle(double regionOrthogonalAngle)
   {
      this.regionOrthogonalAngle = regionOrthogonalAngle;
   }

   public void setSearchHostRegionEpsilon(double searchHostRegionEpsilon)
   {
      this.searchHostRegionEpsilon = searchHostRegionEpsilon;
   }


   @Override
   public double getMaxInterRegionConnectionLength()
   {
      return maxInterRegionConnectionLength;
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return normalZThresholdForAccessibleRegions;
   }

   @Override
   public double getPreferredObstacleExtrusionDistance()
   {
      return preferredExtrusionDistance;
   }

   @Override
   public double getObstacleExtrusionDistance()
   {
      return extrusionDistance;
   }

   @Override
   public double getObstacleExtrusionDistanceIfNotTooHighToStep()
   {
      return extrusionDistanceIfNotTooHighToStep;
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return tooHighToStepDistance;
   }

   @Override
   public double getClusterResolution()
   {
      return clusterResolution;
   }

   @Override
   public double getExplorationDistanceFromStartGoal()
   {
      return explorationDistanceFromStartGoal;
   }

   @Override
   public double getPlanarRegionMinArea()
   {
      return planarRegionMinArea;
   }

   @Override
   public int getPlanarRegionMinSize()
   {
      return planarRegionMinSize;
   }

   @Override
   public double getRegionOrthogonalAngle()
   {
      return regionOrthogonalAngle;
   }

   @Override
   public double getSearchHostRegionEpsilon()
   {
      return searchHostRegionEpsilon;
   }
}
