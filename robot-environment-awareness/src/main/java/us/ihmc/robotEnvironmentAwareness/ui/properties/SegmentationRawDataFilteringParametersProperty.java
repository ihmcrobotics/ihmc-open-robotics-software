package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;

public class SegmentationRawDataFilteringParametersProperty extends ParametersProperty<SegmentationRawDataFilteringParameters>
{
   private DoubleField minimumSparseThreshold = new DoubleField(SegmentationRawDataFilteringParameters::getMinimumSparseThreshold,
                                                                (p, v) -> p.setMinimumSparseThreshold(v));
   private DoubleField maximumSparsePropotionalRatio = new DoubleField(SegmentationRawDataFilteringParameters::getMaximumSparsePropotionalRatio,
                                                                       (p, v) -> p.setMaximumSparsePropotionalRatio(v));

   private BooleanField enableFilterFlyingPoint = new BooleanField(SegmentationRawDataFilteringParameters::isEnableFilterFlyingPoint,
                                                                   (p, v) -> p.setEnableFilterFlyingPoint(v));
   private DoubleField flyingPointThreshold = new DoubleField(SegmentationRawDataFilteringParameters::getFlyingPointThreshold,
                                                              (p, v) -> p.setFlyingPointThreshold(v));
   private IntegerField minimumNumberOfFlyingPointNeighbors = new IntegerField(SegmentationRawDataFilteringParameters::getMinimumNumberOfFlyingPointNeighbors,
                                                                               (p, v) -> p.setMinimumNumberOfFlyingPointNeighbors(v));

   private BooleanField enableFilterCentrality = new BooleanField(SegmentationRawDataFilteringParameters::isEnableFilterCentrality,
                                                                  (p, v) -> p.setEnableFilterCentrality(v));
   private DoubleField centralityRadius = new DoubleField(SegmentationRawDataFilteringParameters::getCentralityRadius, (p, v) -> p.setCentralityRadius(v));
   private DoubleField centralityThreshold = new DoubleField(SegmentationRawDataFilteringParameters::getCentralityThreshold,
                                                             (p, v) -> p.setCentralityThreshold(v));

   private BooleanField enableFilterEllipticity = new BooleanField(SegmentationRawDataFilteringParameters::isEnableFilterEllipticity,
                                                                   (p, v) -> p.setEnableFilterEllipticity(v));
   private DoubleField ellipticityMinimumLength = new DoubleField(SegmentationRawDataFilteringParameters::getEllipticityMinimumLength,
                                                                  (p, v) -> p.setEllipticityMinimumLength(v));
   private DoubleField ellipticityThreshold = new DoubleField(SegmentationRawDataFilteringParameters::getEllipticityThreshold,
                                                              (p, v) -> p.setEllipticityThreshold(v));

   public SegmentationRawDataFilteringParametersProperty(Object bean, String name)
   {
      super(bean, name, new SegmentationRawDataFilteringParameters());
   }

   public void bindBidirectionalSparseThreshold(Property<? extends Number> minimum, Property<? extends Number> propotionalRatio)
   {
      bindFieldBidirectionalToNumberProperty(minimum, minimumSparseThreshold);
      bindFieldBidirectionalToNumberProperty(propotionalRatio, maximumSparsePropotionalRatio);
   }

   public void bindBidirectionalEnableFilterFlyingPoint(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, enableFilterFlyingPoint);
   }

   public void bindBidirectionalFlyingPointParameters(Property<? extends Number> threshold, Property<? extends Number> neighbors)
   {
      bindFieldBidirectionalToNumberProperty(threshold, flyingPointThreshold);
      bindFieldBidirectionalToNumberProperty(neighbors, minimumNumberOfFlyingPointNeighbors);
   }

   public void bindBidirectionalEnableFilterCentrality(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, enableFilterCentrality);
   }

   public void bindBidirectionalCentralityParameters(Property<? extends Number> radius, Property<? extends Number> threshold)
   {
      bindFieldBidirectionalToNumberProperty(radius, centralityRadius);
      bindFieldBidirectionalToNumberProperty(threshold, centralityThreshold);
   }

   public void bindBidirectionalEnableFilterEllipticity(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, enableFilterEllipticity);
   }

   public void bindBidirectionalEllipticityParameters(Property<? extends Number> length, Property<? extends Number> threshold)
   {
      bindFieldBidirectionalToNumberProperty(length, ellipticityMinimumLength);
      bindFieldBidirectionalToNumberProperty(threshold, ellipticityThreshold);
   }

   @Override
   protected SegmentationRawDataFilteringParameters getValueCopy(SegmentationRawDataFilteringParameters valueToCopy)
   {
      return new SegmentationRawDataFilteringParameters(valueToCopy);
   }
}
