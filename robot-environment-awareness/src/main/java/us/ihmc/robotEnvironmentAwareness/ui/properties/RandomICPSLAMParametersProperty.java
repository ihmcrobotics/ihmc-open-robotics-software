package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAMParameters;

public class RandomICPSLAMParametersProperty extends ParametersProperty<RandomICPSLAMParameters>
{
   private final IntegerField numberOfSourcePoints = new IntegerField(RandomICPSLAMParameters::getNumberOfSourcePoints, RandomICPSLAMParameters::setNumberOfSourcePoints);

   private final IntegerField maximumICPSearchingSize = new IntegerField(RandomICPSLAMParameters::getMaximumICPSearchingSize,
                                                                         RandomICPSLAMParameters::setMaximumICPSearchingSize);

   private final DoubleField minimumOverlappedRatio = new DoubleField(RandomICPSLAMParameters::getMinimumOverlappedRatio,
                                                                      RandomICPSLAMParameters::setMinimumOverlappedRatioPercentage);

   private final DoubleField windowSize = new DoubleField(RandomICPSLAMParameters::getWindowMargin, RandomICPSLAMParameters::setWindowMargin);

   private final DoubleField minimumInliersRatioOfKeyFrame = new DoubleField(RandomICPSLAMParameters::getMinimumInliersRatioOfKeyFrame,
                                                                             RandomICPSLAMParameters::setMinimumInliersRatioOfKeyFramePercentage);

   public RandomICPSLAMParametersProperty(Object bean, String name)
   {
      super(bean, name, new RandomICPSLAMParameters());
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
   protected RandomICPSLAMParameters getValueCopy(RandomICPSLAMParameters valueToCopy)
   {
      return new RandomICPSLAMParameters(valueToCopy);
   }
}
