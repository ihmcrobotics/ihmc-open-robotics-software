package us.ihmc.pathPlanning.visibilityGraphs.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class VisibilityGraphsParametersProperty extends ParametersProperty<SettableVisibilityGraphsParameters>
{
   private final DoubleField maxInterRegionConnectionLength = new DoubleField(SettableVisibilityGraphsParameters::getMaxInterRegionConnectionLength, SettableVisibilityGraphsParameters::setMaxInterRegionConnectionLength);
   private final DoubleField normalZThresholdForAccessibleRegions = new DoubleField(SettableVisibilityGraphsParameters::getNormalZThresholdForAccessibleRegions, SettableVisibilityGraphsParameters::setNormalZThresholdForAccessibleRegions);
   private final DoubleField extrusionDistance = new DoubleField(SettableVisibilityGraphsParameters::getObstacleExtrusionDistance, SettableVisibilityGraphsParameters::setExtrusionDistance);
   private final IntegerField planarRegionMinSize = new IntegerField(SettableVisibilityGraphsParameters::getPlanarRegionMinSize, SettableVisibilityGraphsParameters::setPlanarRegionMinSize);
   private final DoubleField extrusionDistanceIfNotTooHighToStep = new DoubleField(SettableVisibilityGraphsParameters::getObstacleExtrusionDistanceIfNotTooHighToStep, SettableVisibilityGraphsParameters::setExtrusionDistanceIfNotTooHighToStep);
   private final DoubleField tooHighToStepDistance = new DoubleField(SettableVisibilityGraphsParameters::getTooHighToStepDistance, SettableVisibilityGraphsParameters::setTooHighToStepDistance);
   private final DoubleField clusterResolution = new DoubleField(SettableVisibilityGraphsParameters::getClusterResolution, SettableVisibilityGraphsParameters::setClusterResolution);
   private final DoubleField explorationDistanceFromStartGoal = new DoubleField(SettableVisibilityGraphsParameters::getExplorationDistanceFromStartGoal, SettableVisibilityGraphsParameters::setExplorationDistanceFromStartGoal);
   private final DoubleField planarRegionMinArea = new DoubleField(SettableVisibilityGraphsParameters::getPlanarRegionMinArea, SettableVisibilityGraphsParameters::setPlanarRegionMinArea);
   private final DoubleField regionOrthogonalAngle = new DoubleField(SettableVisibilityGraphsParameters::getRegionOrthogonalAngle, SettableVisibilityGraphsParameters::setRegionOrthogonalAngle);
   private final DoubleField searchHostRegionEpsilon = new DoubleField(SettableVisibilityGraphsParameters::getSearchHostRegionEpsilon, SettableVisibilityGraphsParameters::setSearchHostRegionEpsilon);

   public VisibilityGraphsParametersProperty(Object bean, String name)
   {
      this(bean, name, new DefaultVisibilityGraphParameters());
   }

   public VisibilityGraphsParametersProperty(Object bean, String name, VisibilityGraphsParameters footstepPlannerParameters)
   {
      super(bean, name, new SettableVisibilityGraphsParameters(footstepPlannerParameters));
   }

   public void setPlannerParameters(VisibilityGraphsParameters parameters)
   {
      setValue(new SettableVisibilityGraphsParameters(parameters));
   }

   @Override
   protected SettableVisibilityGraphsParameters getValueCopy(SettableVisibilityGraphsParameters valueToCopy)
   {
      return new SettableVisibilityGraphsParameters(valueToCopy);
   }

   public void bidirectionalBindMaxInterRegionConnectionLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxInterRegionConnectionLength);
   }

   public void bidirectionalBindNormalZThresholdForAccessibleRegions(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, normalZThresholdForAccessibleRegions);
   }

   public void bidirectionalBindExtrusionDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, extrusionDistance);
   }

   public void bidirectionalBindPlanarRegionMinSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, planarRegionMinSize);
   }

   public void bidirectionalBindExtrusionDistanceIfNotTooHighToStep(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, extrusionDistanceIfNotTooHighToStep);
   }

   public void bidirectionalBindTooHighToStepDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, tooHighToStepDistance);
   }

   public void bidirectionalBindClusterResolution(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, clusterResolution);
   }

   public void bidirectionalBindExplorationDistanceFromStartGoal(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, explorationDistanceFromStartGoal);
   }

   public void bidirectionalBindPlanarRegionMinArea(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, planarRegionMinArea);
   }

   public void bidirectionalBindRegionOrthogonalAngle(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, regionOrthogonalAngle);
   }

   public void bidirectionalBindSearchHostRegionEpsilon(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, searchHostRegionEpsilon);
   }
}
