package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

public class PolygonizerParametersProperty extends ParametersProperty<PolygonizerParameters>
{
   private final DoubleField concaveHullThreshold = new DoubleField(PolygonizerParameters::getConcaveHullThreshold, (p, v) -> p.setConcaveHullThreshold(v));
   private final IntegerField minNumberOfNodes = new IntegerField(PolygonizerParameters::getMinNumberOfNodes, (p, v) -> p.setMinNumberOfNodes(v));
   private final DoubleField shallowAngleThreshold = new DoubleField(PolygonizerParameters::getShallowAngleThreshold, (p, v) -> p.setShallowAngleThreshold(v));
   private final DoubleField peakAngleThreshold = new DoubleField(PolygonizerParameters::getPeakAngleThreshold, (p, v) -> p.setPeakAngleThreshold(v));
   private final DoubleField lengthThreshold = new DoubleField(PolygonizerParameters::getLengthThreshold, (p, v) -> p.setLengthThreshold(v));
   private final DoubleField depthThreshold = new DoubleField(PolygonizerParameters::getDepthThreshold, (p, v) -> p.setDepthThreshold(v));

   public PolygonizerParametersProperty(Object bean, String name)
   {
      super(bean, name, new PolygonizerParameters());
   }

   public void bindBidirectionalConcaveHullThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, concaveHullThreshold);
   }
   
   public void bindBidirectionalMinNumberOfNodes(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minNumberOfNodes);
   }
   
   public void bindBidirectionalShallowAngleThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, shallowAngleThreshold);
   }
   
   public void bindBidirectionalPeakAngleThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, peakAngleThreshold);
   }
   
   public void bindBidirectionalLengthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, lengthThreshold);
   }

   public void bindBidirectionalDepthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, depthThreshold);
   }
   
   @Override
   protected PolygonizerParameters getValueCopy(PolygonizerParameters valueToCopy)
   {
      return new PolygonizerParameters(valueToCopy);
   }
}
