package us.ihmc.pathPlanning.visibilityGraphs.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.VisibilityGraphsParametersProperty.SettableVisibilityGraphsParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class VisibilityGraphsParametersProperty extends ParametersProperty<SettableVisibilityGraphsParameters>
{
   private IntegerField numberOfForcedConnections = new IntegerField(SettableVisibilityGraphsParameters::getNumberOfForcedConnections, (p, v) -> p.setNumberOfForcedConnections(v));
   private DoubleField minimumConnectionDistanceForRegions = new DoubleField(SettableVisibilityGraphsParameters::getMinimumConnectionDistanceForRegions, (p, v) -> p.setMinimumConnectionDistanceForRegions(v));
   private DoubleField normalZThresholdForAccessibleRegions = new DoubleField(SettableVisibilityGraphsParameters::getNormalZThresholdForAccessibleRegions, (p, v) -> p.setNormalZThresholdForAccessibleRegions(v));
   private DoubleField normalZThresholdForPolygonObstacles = new DoubleField(SettableVisibilityGraphsParameters::getNormalZThresholdForPolygonObstacles, (p, v) -> p.setNormalZThresholdForPolygonObstacles(v));
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

   public void binBidirectionalNumberOfForcedConnections(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, numberOfForcedConnections);
   }

   public void binBidirectionalMinimumConnectionDistanceForRegions(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumConnectionDistanceForRegions);
   }

   public void binBidirectionalNormalZThresholdForAccessibleRegions(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, normalZThresholdForAccessibleRegions);
   }

   public void binBidirectionalNormalZThresholdForPolygonObstacles(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, normalZThresholdForPolygonObstacles);
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
      private int numberOfForcedConnections;
      private double minimumConnectionDistanceForRegions;
      private double normalZThresholdForAccessibleRegions;
      private double normalZThresholdForPolygonObstacles;
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
         setNumberOfForcedConnections(parameters.getNumberOfForcedConnections());
         setMinimumConnectionDistanceForRegions(parameters.getMinimumConnectionDistanceForRegions());
         setNormalZThresholdForAccessibleRegions(parameters.getNormalZThresholdForAccessibleRegions());
         setNormalZThresholdForPolygonObstacles(parameters.getNormalZThresholdForPolygonObstacles());
         setExtrusionDistance(parameters.getExtrusionDistance());
         setExtrusionDistanceIfNotTooHighToStep(parameters.getExtrusionDistanceIfNotTooHighToStep());
         setTooHighToStepDistance(parameters.getTooHighToStepDistance());
         setClusterResolution(parameters.getClusterResolution());
         setExplorationDistanceFromStartGoal(parameters.getExplorationDistanceFromStartGoal());
         setPlanarRegionMinArea(parameters.getPlanarRegionMinArea());
         setPlanarRegionMinSize(parameters.getPlanarRegionMinSize());
      }

      @Override
      public int getNumberOfForcedConnections()
      {
         return numberOfForcedConnections;
      }

      @Override
      public double getMinimumConnectionDistanceForRegions()
      {
         return minimumConnectionDistanceForRegions;
      }

      @Override
      public double getNormalZThresholdForAccessibleRegions()
      {
         return normalZThresholdForAccessibleRegions;
      }

      @Override
      public double getNormalZThresholdForPolygonObstacles()
      {
         return normalZThresholdForPolygonObstacles;
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

      public void setNumberOfForcedConnections(int numberOfForcedConnections)
      {
         this.numberOfForcedConnections = numberOfForcedConnections;
      }

      public void setMinimumConnectionDistanceForRegions(double minimumConnectionDistanceForRegions)
      {
         this.minimumConnectionDistanceForRegions = minimumConnectionDistanceForRegions;
      }

      public void setNormalZThresholdForAccessibleRegions(double normalZThresholdForAccessibleRegions)
      {
         this.normalZThresholdForAccessibleRegions = normalZThresholdForAccessibleRegions;
      }

      public void setNormalZThresholdForPolygonObstacles(double normalZThresholdForPolygonObstacles)
      {
         this.normalZThresholdForPolygonObstacles = normalZThresholdForPolygonObstacles;
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
