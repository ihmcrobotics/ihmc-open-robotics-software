package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IhmcSLAMParameters;

public class IhmcSLAMParametersProperty extends ParametersProperty<IhmcSLAMParameters>
{
   private final DoubleField octreeResolution = new DoubleField(IhmcSLAMParameters::getOctreeResolution, IhmcSLAMParameters::setOctreeResolution);
   private final IntegerField numberOfSourcePoints = new IntegerField(IhmcSLAMParameters::getNumberOfSourcePoints, IhmcSLAMParameters::setNumberOfSourcePoints);
   private final DoubleField maximumDepth = new DoubleField(IhmcSLAMParameters::getMaximumDepth, IhmcSLAMParameters::setMaximumDepth);
   private final DoubleField minimumDepth = new DoubleField(IhmcSLAMParameters::getMinimumDepth, IhmcSLAMParameters::setMinimumDepth);
   private final DoubleField minimumOverlappedRatio = new DoubleField(IhmcSLAMParameters::getMinimumOverlappedRatio,
                                                                      IhmcSLAMParameters::setMinimumOverlappedRatio);

   private final IntegerField maximumICPSearchingSize = new IntegerField(IhmcSLAMParameters::getMaximumICPSearchingSize,
                                                                         IhmcSLAMParameters::setMaximumICPSearchingSize);

   private final DoubleField minimumInliersRatioOfKeyFrame = new DoubleField(IhmcSLAMParameters::getMinimumInliersRatioOfKeyFrame,
                                                                             IhmcSLAMParameters::setMinimumInliersRatioOfKeyFrame);

   public IhmcSLAMParametersProperty(Object bean, String name)
   {
      super(bean, name, new IhmcSLAMParameters());
   }

   public void bindBidirectionalOctreeResolution(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, octreeResolution);
   }

   public void bindBidirectionalNumberOfSourcePoints(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, numberOfSourcePoints);
   }

   public void bindBidirectionalDepthBoundary(Property<? extends Number> lower, Property<? extends Number> upper)
   {
      bindFieldBidirectionalToNumberProperty(lower, minimumDepth);
      bindFieldBidirectionalToNumberProperty(upper, maximumDepth);
   }

   public void bindBidirectionalMinimumOverlappedRatio(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumOverlappedRatio);
   }

   public void bindBidirectionalMaximumICPSearchingSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumICPSearchingSize);
   }

   public void bindBidirectionalMinimumInliersRatioOfKeyFrame(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumInliersRatioOfKeyFrame);
   }

   @Override
   protected IhmcSLAMParameters getValueCopy(IhmcSLAMParameters valueToCopy)
   {
      return new IhmcSLAMParameters(valueToCopy);
   }
}
