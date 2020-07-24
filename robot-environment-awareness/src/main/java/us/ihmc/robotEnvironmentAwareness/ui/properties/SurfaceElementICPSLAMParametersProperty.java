package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAMParameters;

public class SurfaceElementICPSLAMParametersProperty extends ParametersProperty<SurfaceElementICPSLAMParameters>
{
   private final DoubleField surfaceElementResolution = new DoubleField(SurfaceElementICPSLAMParameters::getSurfaceElementResolution,
                                                                        SurfaceElementICPSLAMParameters::setSurfaceElementResolution);
   private final DoubleField windowMargin = new DoubleField(SurfaceElementICPSLAMParameters::getWindowMargin, SurfaceElementICPSLAMParameters::setWindowMargin);
   private final IntegerField minimumNumberOfHit = new IntegerField(SurfaceElementICPSLAMParameters::getMinimumNumberOfHit,
                                                                    SurfaceElementICPSLAMParameters::setMinimumNumberOfHit);
   private final DoubleField boundRatio = new DoubleField(SurfaceElementICPSLAMParameters::getBoundRatio, SurfaceElementICPSLAMParameters::setBoundRatio);

   private final DoubleField minimumCorrespondingDistance = new DoubleField(SurfaceElementICPSLAMParameters::getMinimumCorrespondingDistance,
                                                                            SurfaceElementICPSLAMParameters::setMinimumCorrespondingDistance);
   private final DoubleField maximumCorrespondingDistance = new DoubleField(SurfaceElementICPSLAMParameters::getMaximumCorrespondingDistance,
                                                                            SurfaceElementICPSLAMParameters::setMaximumCorrespondingDistance);
   private final IntegerField constantCorrespondingDistanceIteration = new IntegerField(SurfaceElementICPSLAMParameters::getConstantCorrespondingDistanceIteration,
                                                                                        SurfaceElementICPSLAMParameters::setConstantCorrespondingDistanceIteration);

   private final IntegerField steadyStateDetectorIterationThreshold = new IntegerField(SurfaceElementICPSLAMParameters::getSteadyStateDetectorIterationThreshold,
                                                                                       SurfaceElementICPSLAMParameters::setSteadyStateDetectorIterationThreshold);
   private final DoubleField qualityConvergenceThreshold = new DoubleField(SurfaceElementICPSLAMParameters::getQualityConvergenceThreshold,
                                                                           SurfaceElementICPSLAMParameters::setQualityConvergenceThreshold);
   private final DoubleField translationalEffortConvergenceThreshold = new DoubleField(SurfaceElementICPSLAMParameters::getTranslationalEffortConvergenceThreshold,
                                                                                       SurfaceElementICPSLAMParameters::setTranslationalEffortConvergenceThreshold);
   private final DoubleField rotationalEffortConvergenceThreshold = new DoubleField(SurfaceElementICPSLAMParameters::getRotationalEffortConvergenceThreshold,
                                                                                    SurfaceElementICPSLAMParameters::setRotationalEffortConvergenceThreshold);

   private final BooleanField enableInitialQualityFilter = new BooleanField(SurfaceElementICPSLAMParameters::isEnableInitialQualityFilter,
                                                                            SurfaceElementICPSLAMParameters::setEnableInitialQualityFilter);

   private final DoubleField initialQualityThreshold = new DoubleField(SurfaceElementICPSLAMParameters::getInitialQualityThreshold,
                                                                       SurfaceElementICPSLAMParameters::setInitialQualityThreshold);

   public SurfaceElementICPSLAMParametersProperty(Object bean, String name)
   {
      super(bean, name, new SurfaceElementICPSLAMParameters());
   }

   public void bindBidirectionalSurfaceElementResolution(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, surfaceElementResolution);
   }

   public void bindBidirectionalWindowMargin(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, windowMargin);
   }

   public void bindBidirectionalMinimumNumberOfHit(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumNumberOfHit);
   }

   public void bindBidirectionalBoundRatio(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, boundRatio);
   }

   public void bindBidirectionalMinimumCorrespondingDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minimumCorrespondingDistance);
   }

   public void bindBidirectionalMaximumCorrespondingDistance(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maximumCorrespondingDistance);
   }

   public void bindBidirectionalConstantCorrespondingDistanceIteration(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, constantCorrespondingDistanceIteration);
   }

   public void bindBidirectionalSteadyStateDetectorIterationThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, steadyStateDetectorIterationThreshold);
   }

   public void bindBidirectionalQualityConvergenceThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, qualityConvergenceThreshold);
   }

   public void bindBidirectionalTranslationalEffortConvergenceThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, translationalEffortConvergenceThreshold);
   }

   public void bindBidirectionalRotationalEffortConvergenceThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, rotationalEffortConvergenceThreshold);
   }

   public void bindBidirectionalEnableInitialQualityFilter(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, enableInitialQualityFilter);
   }

   public void bindBidirectionalInitialQualityThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, initialQualityThreshold);
   }

   @Override
   protected SurfaceElementICPSLAMParameters getValueCopy(SurfaceElementICPSLAMParameters valueToCopy)
   {
      return new SurfaceElementICPSLAMParameters(valueToCopy);
   }
}
