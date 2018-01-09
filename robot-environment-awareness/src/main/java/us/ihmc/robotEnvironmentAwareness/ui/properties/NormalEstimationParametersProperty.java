package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;

public class NormalEstimationParametersProperty extends ParametersProperty<NormalEstimationParameters>
{
   private final DoubleField searchRadius = new DoubleField(NormalEstimationParameters::getSearchRadius, (p, v) -> p.setSearchRadius(v));
   private final DoubleField maxDistanceFromPlane = new DoubleField(NormalEstimationParameters::getMaxDistanceFromPlane, (p, v) -> p.setMaxDistanceFromPlane(v));
   private final DoubleField minConsensusRatio = new DoubleField(NormalEstimationParameters::getMinConsensusRatio, (p, v) -> p.setMinConsensusRatio(v));
   private final DoubleField maxAverageDeviationRatio = new DoubleField(NormalEstimationParameters::getMaxAverageDeviationRatio, (p, v) -> p.setMaxAverageDeviationRatio(v));
   private final IntegerField numberOfIterations = new IntegerField(NormalEstimationParameters::getNumberOfIterations, (p, v) -> p.setNumberOfIterations(v));

   public NormalEstimationParametersProperty(Object bean, String name)
   {
      super(bean, name, new NormalEstimationParameters());
   }

   public void bindBidirectionalSearchRadius(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, searchRadius);
   }

   public void bindBidirectionalMaxDistanceFromPlane(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxDistanceFromPlane);
   }

   public void bindBidirectionalMinConsensusRatio(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minConsensusRatio);
   }

   public void bindBidirectionalMaxAverageDeviationRatio(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxAverageDeviationRatio);
   }

   public void bindBidirectionalNumberOfIterations(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, numberOfIterations);
   }

   @Override
   protected NormalEstimationParameters getValueCopy(NormalEstimationParameters valueToCopy)
   {
      return new NormalEstimationParameters(valueToCopy);
   }
}
