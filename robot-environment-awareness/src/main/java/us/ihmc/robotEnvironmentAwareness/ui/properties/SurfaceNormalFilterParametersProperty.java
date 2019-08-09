package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;

public class SurfaceNormalFilterParametersProperty extends ParametersProperty<SurfaceNormalFilterParameters>
{
   private final BooleanField useFilter = new BooleanField(SurfaceNormalFilterParameters::isUseSurfaceNormalFilter, (p, v) -> p.setUseSurfaceNormalFilter(v));
   private final DoubleField upperBound = new DoubleField(SurfaceNormalFilterParameters::getSurfaceNormalUpperBoundDegree,
                                                          (p, v) -> p.setSurfaceNormalUpperBoundDegree(v));
   private final DoubleField lowerBound = new DoubleField(SurfaceNormalFilterParameters::getSurfaceNormalLowerBoundDegree,
                                                          (p, v) -> p.setSurfaceNormalLowerBoundDegree(v));

   public SurfaceNormalFilterParametersProperty(Object bean, String name)
   {
      super(bean, name, new SurfaceNormalFilterParameters());
   }

   public void bindBidirectionalBounds(Property<? extends Number> upperProperty, Property<? extends Number> lowerProperty)
   {
      bindFieldBidirectionalToNumberProperty(upperProperty, upperBound);
      bindFieldBidirectionalToNumberProperty(lowerProperty, lowerBound);
   }

   public void bindBidirectionalUseFilter(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, useFilter);
   }

   @Override
   protected SurfaceNormalFilterParameters getValueCopy(SurfaceNormalFilterParameters valueToCopy)
   {
      return new SurfaceNormalFilterParameters(valueToCopy);
   }
}
