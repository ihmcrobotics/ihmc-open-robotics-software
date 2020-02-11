package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.slam.IhmcSLAMParameters;

public class IhmcSLAMParametersProperty extends ParametersProperty<IhmcSLAMParameters>
{
   private final IntegerField numberOfSourcePoints = new IntegerField(IhmcSLAMParameters::getNumberOfSourcePoints, IhmcSLAMParameters::setNumberOfSourcePoints);

   private final IntegerField maximumICPSearchingSize = new IntegerField(IhmcSLAMParameters::getMaximumICPSearchingSize,
                                                                         IhmcSLAMParameters::setMaximumICPSearchingSize);

   private final DoubleField minimumOverlappedRatio = new DoubleField(IhmcSLAMParameters::getMinimumOverlappedRatio,
                                                                      IhmcSLAMParameters::setMinimumOverlappedRatioPercentage);

   private final DoubleField windowSize = new DoubleField(IhmcSLAMParameters::getWindowMargin, IhmcSLAMParameters::setWindowMargin);

   private final DoubleField minimumInliersRatioOfKeyFrame = new DoubleField(IhmcSLAMParameters::getMinimumInliersRatioOfKeyFrame,
                                                                             IhmcSLAMParameters::setMinimumInliersRatioOfKeyFramePercentage);

   public IhmcSLAMParametersProperty(Object bean, String name)
   {
      super(bean, name, new IhmcSLAMParameters());
   }

   public void bindBidirectionalNumberOfSourcePoints(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, numberOfSourcePoints);
   }

   public void bindBidirectionalMinimumOverlappedRatio(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumOverlappedRatio);
   }

   public void bindBidirectionalWindowSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, windowSize);
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
