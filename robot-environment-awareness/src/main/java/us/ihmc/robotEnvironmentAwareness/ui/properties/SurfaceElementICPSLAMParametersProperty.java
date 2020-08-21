package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
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

   private final IntegerField maxQueueSize = new IntegerField(SurfaceElementICPSLAMParameters::getMaximumQueueSize,
                                                              SurfaceElementICPSLAMParameters::setMaximumQueueSize);

   private final DoubleField longestTimeToLag = new DoubleField(SurfaceElementICPSLAMParameters::getLongestTimeToLag,
                                                                SurfaceElementICPSLAMParameters::setLongestTimeToLag);

   private final IntegerField maxOptimizationIterations = new IntegerField(SurfaceElementICPSLAMParameters::getMaxOptimizationIterations,
                                                                           SurfaceElementICPSLAMParameters::setMaxOptimizationIterations);
   private final BooleanField computeSurfaceNormalsInFrame = new BooleanField(SurfaceElementICPSLAMParameters::getComputeSurfaceNormalsInFrame,
                                                                              SurfaceElementICPSLAMParameters::setComputeSurfaceNormalsInFrame);
   private final BooleanField insertMissInOcTree = new BooleanField(SurfaceElementICPSLAMParameters::getInsertMissInOcTree,
                                                                    SurfaceElementICPSLAMParameters::setInsertMissInOcTree);
   private final BooleanField computeFramesInParallel = new BooleanField(SurfaceElementICPSLAMParameters::getComputeFramesInParallel,
                                                                         SurfaceElementICPSLAMParameters::setComputeFramesInParallel);

   private final BooleanField includePitchAndRoll = new BooleanField(SurfaceElementICPSLAMParameters::getIncludePitchAndRoll,
                                                                     SurfaceElementICPSLAMParameters::setIncludePitchAndRoll);
   private final DoubleField translationPerturbation = new DoubleField(SurfaceElementICPSLAMParameters::getTranslationPerturbation,
                                                                       SurfaceElementICPSLAMParameters::setTranslationPerturbation);
   private final DoubleField rotationPerturbation = new DoubleField(SurfaceElementICPSLAMParameters::getRotationPerturbation,
                                                                    SurfaceElementICPSLAMParameters::setRotationPerturbation);

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

   public void bindBidirectionalEnableInitialQualityFilter(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, enableInitialQualityFilter);
   }

   public void bindBidirectionalInitialQualityThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, initialQualityThreshold);
   }

   public void bindBidirectionalMaxQueueSize(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxQueueSize);
   }

   public void bindBidirectionalLongestTimeToLag(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, longestTimeToLag);
   }

   public void bindBidirectionalMaxOptimizationIterations(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxOptimizationIterations);
   }

   public void bindBidirectionalComputeSurfaceNormalsInFrame(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, computeSurfaceNormalsInFrame);
   }

   public void bindBidirectionalInsertMissInOcTree(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, insertMissInOcTree);
   }

   public void bindBidirectionalComputeFramesInParallel(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, computeFramesInParallel);
   }

   public void bindBidirectionalIncludePitchAndRoll(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, includePitchAndRoll);
   }

   public void bindBidirectionalTranslationPerturbation(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, translationPerturbation);
   }

   public void bindBidirectionalRotationPerturbation(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, rotationPerturbation);
   }

   @Override
   protected SurfaceElementICPSLAMParameters getValueCopy(SurfaceElementICPSLAMParameters valueToCopy)
   {
      return new SurfaceElementICPSLAMParameters(valueToCopy);
   }
}
