package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;

public class CustomRegionMergeParametersProperty extends ParametersProperty<CustomRegionMergeParameters>
{
   private final DoubleField searchRadius = new DoubleField(CustomRegionMergeParameters::getSearchRadius, CustomRegionMergeParameters::setSearchRadius);
   private final DoubleField maxDistanceFromPlane = new DoubleField(CustomRegionMergeParameters::getMaxDistanceFromPlane, CustomRegionMergeParameters::setMaxDistanceFromPlane);
   private final DoubleField maxAngleFromPlane = new DoubleField(CustomRegionMergeParameters::getMaxAngleFromPlane, CustomRegionMergeParameters::setMaxAngleFromPlane);

   public CustomRegionMergeParametersProperty(Object bean, String name)
   {
      super(bean, name, new CustomRegionMergeParameters());
   }

   public void bindBidirectionalSearchRadius(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, searchRadius);
   }

   public void bindBidirectionalMaxDistanceFromPlane(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxDistanceFromPlane);
   }

   public void bindBidirectionalMaxAngleFromPlane(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxAngleFromPlane);
   }

   @Override
   protected CustomRegionMergeParameters getValueCopy(CustomRegionMergeParameters valueToCopy)
   {
      return new CustomRegionMergeParameters(valueToCopy);
   }
}
