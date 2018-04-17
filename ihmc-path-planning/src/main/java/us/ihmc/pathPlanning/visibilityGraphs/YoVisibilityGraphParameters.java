package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoVisibilityGraphParameters implements VisibilityGraphsParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry("VisibilityGraphParameters");

   private final YoDouble maxInterRegionConnectionLength = new YoDouble("maxInterRegionConnectionLength", registry);
   private final YoDouble normalZThresholdForAccessibleRegions = new YoDouble("normalZThresholdForAccessibleRegions", registry);
   private final YoDouble maxAngleBeforeOrthogonal = new YoDouble("normalZThresholdForPolygonObstacles", registry);
   private final YoDouble extrusionDistance = new YoDouble("extrusionDistance", registry);
   private final YoDouble extrusionDistanceIfNotTooHighToStep = new YoDouble("extrusionDistanceIfNotTooHighToStep", registry);
   private final YoDouble tooHighToStepDistance = new YoDouble("tooHighToStepDistance", registry);
   private final YoDouble clusterResolution = new YoDouble("clusterResolution", registry);
   private final YoDouble explorationDistance = new YoDouble("explorationDistance", registry);

   public YoVisibilityGraphParameters(VisibilityGraphsParameters defaults, YoVariableRegistry parentRegistry)
   {
      this.maxInterRegionConnectionLength.set(defaults.getMaxInterRegionConnectionLength());
      this.normalZThresholdForAccessibleRegions.set(defaults.getNormalZThresholdForAccessibleRegions());
      this.maxAngleBeforeOrthogonal.set(defaults.getRegionOrthogonalAngle());
      this.extrusionDistance.set(defaults.getExtrusionDistance());
      this.extrusionDistanceIfNotTooHighToStep.set(defaults.getExtrusionDistanceIfNotTooHighToStep());
      this.tooHighToStepDistance.set(defaults.getTooHighToStepDistance());
      this.clusterResolution.set(defaults.getClusterResolution());
      this.explorationDistance.set(defaults.getExplorationDistanceFromStartGoal());

      parentRegistry.addChild(registry);
   }

   @Override
   public double getMaxInterRegionConnectionLength()
   {
      return maxInterRegionConnectionLength.getDoubleValue();
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return normalZThresholdForAccessibleRegions.getDoubleValue();
   }

   @Override
   public double getRegionOrthogonalAngle()
   {
      return maxAngleBeforeOrthogonal.getDoubleValue();
   }

   @Override
   public double getExtrusionDistance()
   {
      return extrusionDistance.getDoubleValue();
   }

   @Override
   public double getExtrusionDistanceIfNotTooHighToStep()
   {
      return extrusionDistanceIfNotTooHighToStep.getDoubleValue();
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return tooHighToStepDistance.getDoubleValue();
   }

   @Override
   public double getClusterResolution()
   {
      return clusterResolution.getDoubleValue();
   }

   @Override
   public double getExplorationDistanceFromStartGoal()
   {
      return explorationDistance.getDoubleValue();
   }
}
