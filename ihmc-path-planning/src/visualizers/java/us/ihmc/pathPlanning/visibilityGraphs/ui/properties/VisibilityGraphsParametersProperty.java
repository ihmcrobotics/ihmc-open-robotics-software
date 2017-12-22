package us.ihmc.pathPlanning.visibilityGraphs.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.VisibilityGraphsParametersProperty.SettableVisibilityGraphsParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class VisibilityGraphsParametersProperty extends ParametersProperty<SettableVisibilityGraphsParameters>
{
   private DoubleField maxInterRegionConnectionLength = new DoubleField(SettableVisibilityGraphsParameters::getMaxInterRegionConnectionLength, (p, v) -> p.setMaxInterRegionConnectionLength(v));
   private DoubleField normalZThresholdForAccessibleRegions = new DoubleField(SettableVisibilityGraphsParameters::getNormalZThresholdForAccessibleRegions, (p, v) -> p.setNormalZThresholdForAccessibleRegions(v));
   private DoubleField regionOrthogonalAngle = new DoubleField(SettableVisibilityGraphsParameters::getRegionOrthogonalAngle, (p, v) -> p.setRegionOrthogonalAngle(v));
   private DoubleField extrusionDistance = new DoubleField(SettableVisibilityGraphsParameters::getExtrusionDistance, (p, v) -> p.setExtrusionDistance(v));
   private DoubleField extrusionDistanceIfNotTooHighToStep = new DoubleField(SettableVisibilityGraphsParameters::getExtrusionDistanceIfNotTooHighToStep, (p, v) -> p.setExtrusionDistanceIfNotTooHighToStep(v));
   private DoubleField tooHighToStepDistance = new DoubleField(SettableVisibilityGraphsParameters::getTooHighToStepDistance, (p, v) -> p.setTooHighToStepDistance(v));
   private DoubleField clusterResolution = new DoubleField(SettableVisibilityGraphsParameters::getClusterResolution, (p, v) -> p.setClusterResolution(v));
   private DoubleField explorationDistanceFromStartGoal = new DoubleField(SettableVisibilityGraphsParameters::getExplorationDistanceFromStartGoal, (p, v) -> p.setExplorationDistanceFromStartGoal(v));
   private DoubleField planarRegionMinArea = new DoubleField(SettableVisibilityGraphsParameters::getPlanarRegionMinArea, (p, v) -> p.setPlanarRegionMinArea(v));
   private IntegerField planarRegionMinSize = new IntegerField(SettableVisibilityGraphsParameters::getPlanarRegionMinSize, (p, v) -> p.setPlanarRegionMinSize(v));

   public VisibilityGraphsParametersProperty(Object bean, String name)
   {
      super(bean, name, new SettableVisibilityGraphsParameters());
   }

   public void binBidirectionalMaxInterRegionConnectionLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxInterRegionConnectionLength);
   }

   public void binBidirectionalNormalZThresholdForAccessibleRegions(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, normalZThresholdForAccessibleRegions);
   }

   public void binBidirectionalRegionOrthogonalAngle(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, regionOrthogonalAngle);
   }

   public void binBidirectionalExtrusionDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, extrusionDistance);
   }

   public void binBidirectionalExtrusionDistanceIfNotTooHighToStep(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, extrusionDistanceIfNotTooHighToStep);
   }

   public void binBidirectionalTooHighToStepDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, tooHighToStepDistance);
   }

   public void binBidirectionalClusterResolution(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, clusterResolution);
   }

   public void binBidirectionalExplorationDistanceFromStartGoal(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, explorationDistanceFromStartGoal);
   }

   public void binBidirectionalPlanarRegionMinArea(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, planarRegionMinArea);
   }

   public void binBidirectionalPlanarRegionMinSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, planarRegionMinSize);
   }

   @Override
   protected SettableVisibilityGraphsParameters getValueCopy(SettableVisibilityGraphsParameters valueToCopy)
   {
      return new SettableVisibilityGraphsParameters(valueToCopy);
   }

   public static class SettableVisibilityGraphsParameters implements VisibilityGraphsParameters
   {
      private double maxInterRegionConnectionLength;
      private double normalZThresholdForAccessibleRegions;
      private double regionOrthogonalAngle;
      private double extrusionDistance;
      private double extrusionDistanceIfNotTooHighToStep;
      private double tooHighToStepDistance;
      private double clusterResolution;
      private double explorationDistanceFromStartGoal;
      private double planarRegionMinArea;
      private int planarRegionMinSize;

      public SettableVisibilityGraphsParameters()
      {
         this(new DefaultVisibilityGraphParameters());
      }

      public SettableVisibilityGraphsParameters(VisibilityGraphsParameters parameters)
      {
         setMaxInterRegionConnectionLength(parameters.getMaxInterRegionConnectionLength());
         setNormalZThresholdForAccessibleRegions(parameters.getNormalZThresholdForAccessibleRegions());
         setRegionOrthogonalAngle(parameters.getRegionOrthogonalAngle());
         setExtrusionDistance(parameters.getExtrusionDistance());
         setExtrusionDistanceIfNotTooHighToStep(parameters.getExtrusionDistanceIfNotTooHighToStep());
         setTooHighToStepDistance(parameters.getTooHighToStepDistance());
         setClusterResolution(parameters.getClusterResolution());
         setExplorationDistanceFromStartGoal(parameters.getExplorationDistanceFromStartGoal());
         setPlanarRegionMinArea(parameters.getPlanarRegionMinArea());
         setPlanarRegionMinSize(parameters.getPlanarRegionMinSize());
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
      public double getRegionOrthogonalAngle()
      {
         return regionOrthogonalAngle;
      }

      @Override
      public double getExtrusionDistance()
      {
         return extrusionDistance;
      }

      @Override
      public double getExtrusionDistanceIfNotTooHighToStep()
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

      public void setMaxInterRegionConnectionLength(double maxInterRegionConnectionLength)
      {
         this.maxInterRegionConnectionLength = maxInterRegionConnectionLength;
      }

      public void setNormalZThresholdForAccessibleRegions(double normalZThresholdForAccessibleRegions)
      {
         this.normalZThresholdForAccessibleRegions = normalZThresholdForAccessibleRegions;
      }

      public void setRegionOrthogonalAngle(double normalZThresholdForPolygonObstacles)
      {
         this.regionOrthogonalAngle = normalZThresholdForPolygonObstacles;
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
   }
}
