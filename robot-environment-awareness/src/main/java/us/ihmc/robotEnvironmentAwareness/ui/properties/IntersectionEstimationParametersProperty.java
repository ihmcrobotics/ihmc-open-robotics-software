package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;

public class IntersectionEstimationParametersProperty extends ParametersProperty<IntersectionEstimationParameters>
{
   private final DoubleField maxDistanceToRegion = new DoubleField(IntersectionEstimationParameters::getMaxDistanceToRegion, (p, v) -> p.setMaxDistanceToRegion(v));
   private final IntegerField minRegionSize = new IntegerField(IntersectionEstimationParameters::getMinRegionSize, (p, v) -> p.setMinRegionSize(v));
   private final DoubleField minIntersectionLength = new DoubleField(IntersectionEstimationParameters::getMinIntersectionLength, (p, v) -> p.setMinIntersectionLength(v));
   private final DoubleField minRegionAngleDifference = new DoubleField(IntersectionEstimationParameters::getMinRegionAngleDifference, (p, v) -> p.setMinRegionAngleDifference(v));
   private final BooleanField addIntersectionsToRegions = new BooleanField(IntersectionEstimationParameters::isAddIntersectionsToRegions, (p, v) -> p.setAddIntersectionsToRegions(v));

   public IntersectionEstimationParametersProperty(Object bean, String name)
   {
      super(bean, name, new IntersectionEstimationParameters());
   }

   public void bindBidirectionalMaxDistanceToRegion(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxDistanceToRegion);
   }

   public void bindBidirectionalMinRegionSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minRegionSize);
   }

   public void bindBidirectionalMinIntersectionLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minIntersectionLength);
   }

   public void bindBidirectionalMinRegionAngleDifference(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minRegionAngleDifference);
   }

   public void bindBidirectionalAddIntersectionsToRegions(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, addIntersectionsToRegions);
   }

   @Override
   protected IntersectionEstimationParameters getValueCopy(IntersectionEstimationParameters valueToCopy)
   {
      return new IntersectionEstimationParameters(valueToCopy);
   }
}
