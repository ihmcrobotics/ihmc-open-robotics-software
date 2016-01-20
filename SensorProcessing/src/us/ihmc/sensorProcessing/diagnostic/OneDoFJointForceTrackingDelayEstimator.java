package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class OneDoFJointForceTrackingDelayEstimator implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;
   private final OneDoFJoint joint;
   private final DelayEstimatorBetweenTwoSignals delayEstimator;

   public OneDoFJointForceTrackingDelayEstimator(OneDoFJoint joint, double dt, YoVariableRegistry parentRegistry)
   {
      this.joint = joint;
      String jointName = joint.getName();
      registry = new YoVariableRegistry(jointName + "ForceTrackingDelayEstimator");
      delayEstimator = new DelayEstimatorBetweenTwoSignals(jointName + "ForceTracking", dt, registry);
   }

   @Override
   public void enable()
   {
      delayEstimator.enable();
   }

   @Override
   public void disable()
   {
      delayEstimator.disable();
   }

   @Override
   public void update()
   {
      delayEstimator.update(joint.getTau(), joint.getTauMeasured());
   }

   public void setAlphaFilterBreakFrequency(double delayEstimatorFilterBreakFrequency)
   {
      delayEstimator.setAlphaFilterBreakFrequency(delayEstimatorFilterBreakFrequency);
   }

   public void setEstimationParameters(double maxAbsoluteLead, double maxAbsoluteLag, double observationWindow)
   {
      delayEstimator.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
   }

   public boolean isEstimatingDelay()
   {
      return delayEstimator.isEstimatingDelay();
   }

   public double getEstimatedDelay()
   {
      return delayEstimator.getEstimatedDelay();
   }

   public double getCorrelation()
   {
      return delayEstimator.getCorrelationCoefficient();
   }
}
