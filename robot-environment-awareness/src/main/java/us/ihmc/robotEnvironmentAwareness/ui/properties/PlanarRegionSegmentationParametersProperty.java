package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;

public class PlanarRegionSegmentationParametersProperty extends ParametersProperty<PlanarRegionSegmentationParameters>
{
   private final DoubleField searchRadius = new DoubleField(PlanarRegionSegmentationParameters::getSearchRadius, (p, v) -> p.setSearchRadius(v));
   private final DoubleField maxDistanceFromPlane = new DoubleField(PlanarRegionSegmentationParameters::getMaxDistanceFromPlane, (p, v) -> p.setMaxDistanceFromPlane(v));
   private final DoubleField maxAngleFromPlane = new DoubleField(PlanarRegionSegmentationParameters::getMaxAngleFromPlane, (p, v) -> p.setMaxAngleFromPlane(v));
   private final DoubleField minNormalQuality = new DoubleField(PlanarRegionSegmentationParameters::getMinNormalQuality, (p, v) -> p.setMinNormalQuality(v));
   private final IntegerField minRegionSize = new IntegerField(PlanarRegionSegmentationParameters::getMinRegionSize, (p, v) -> p.setMinRegionSize(v));
   private final DoubleField maxStandardDeviation = new DoubleField(PlanarRegionSegmentationParameters::getMaxStandardDeviation, (p, v) -> p.setMaxStandardDeviation(v));
   private final DoubleField minVolumicDensity = new DoubleField(PlanarRegionSegmentationParameters::getMinVolumicDensity, (p, v) -> p.setMinVolumicDensity(v));

   public PlanarRegionSegmentationParametersProperty(Object bean, String name)
   {
      super(bean, name, new PlanarRegionSegmentationParameters());
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

   public void bindBidirectionalMinNormalQuality(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minNormalQuality);
   }

   public void bindBidirectionalMinRegionSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minRegionSize);
   }

   public void bindBidirectionalMaxStandardDeviation(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStandardDeviation);
   }

   public void bindBidirectionalMinVolumicDensity(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minVolumicDensity);
   }

   @Override
   protected PlanarRegionSegmentationParameters getValueCopy(PlanarRegionSegmentationParameters valueToCopy)
   {
      return new PlanarRegionSegmentationParameters(valueToCopy);
   }
}
