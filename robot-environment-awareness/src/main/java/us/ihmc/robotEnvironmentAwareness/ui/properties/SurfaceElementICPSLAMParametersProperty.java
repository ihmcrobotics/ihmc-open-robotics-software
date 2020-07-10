package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAMParameters;

public class SurfaceElementICPSLAMParametersProperty extends ParametersProperty<SurfaceElementICPSLAMParameters>
{
//   private final IntegerField numberOfSourcePoints = new IntegerField(SurfaceElementICPSLAMParameters::getNumberOfSourcePoints, SurfaceElementICPSLAMParameters::setNumberOfSourcePoints);
//
//   private final IntegerField maximumICPSearchingSize = new IntegerField(SurfaceElementICPSLAMParameters::getMaximumICPSearchingSize,
//                                                                         SurfaceElementICPSLAMParameters::setMaximumICPSearchingSize);
//
//   private final DoubleField minimumOverlappedRatio = new DoubleField(SurfaceElementICPSLAMParameters::getMinimumOverlappedRatio,
//                                                                      SurfaceElementICPSLAMParameters::setMinimumOverlappedRatioPercentage);
//
//   private final DoubleField windowSize = new DoubleField(SurfaceElementICPSLAMParameters::getWindowMargin, SurfaceElementICPSLAMParameters::setWindowMargin);
//
//   private final DoubleField minimumInliersRatioOfKeyFrame = new DoubleField(SurfaceElementICPSLAMParameters::getMinimumInliersRatioOfKeyFrame,
//                                                                             SurfaceElementICPSLAMParameters::setMinimumInliersRatioOfKeyFramePercentage);

   public SurfaceElementICPSLAMParametersProperty(Object bean, String name)
   {
      super(bean, name, new SurfaceElementICPSLAMParameters());
   }

//   public void bindBidirectionalNumberOfSourcePoints(Property<? extends Number> property)
//   {
//      bindFieldBidirectionalToNumberProperty(property, numberOfSourcePoints);
//   }
//
//   public void bindBidirectionalMinimumOverlappedRatio(Property<? extends Number> property)
//   {
//      bindFieldBidirectionalToNumberProperty(property, minimumOverlappedRatio);
//   }
//
//   public void bindBidirectionalWindowSize(Property<? extends Number> property)
//   {
//      bindFieldBidirectionalToNumberProperty(property, windowSize);
//   }
//
//   public void bindBidirectionalMaximumICPSearchingSize(Property<? extends Number> property)
//   {
//      bindFieldBidirectionalToNumberProperty(property, maximumICPSearchingSize);
//   }
//
//   public void bindBidirectionalMinimumInliersRatioOfKeyFrame(Property<? extends Number> property)
//   {
//      bindFieldBidirectionalToNumberProperty(property, minimumInliersRatioOfKeyFrame);
//   }

   @Override
   protected SurfaceElementICPSLAMParameters getValueCopy(SurfaceElementICPSLAMParameters valueToCopy)
   {
      return new SurfaceElementICPSLAMParameters(valueToCopy);
   }
}
