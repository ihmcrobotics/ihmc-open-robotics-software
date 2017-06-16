package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Defines parameters to use in the EuclideanTangentialDampingCalculator. This reduces the damping ratio when incurring large tracking errors.
 */
public class YoTangentialDampingGains implements TangentialDampingGains
{
   private final YoDouble kdParallelMaxReductionRatio;
   private final YoDouble dampingParallelToMotionDeadband;
   private final YoDouble positionErrorForMinimumKd;

   public YoTangentialDampingGains(String suffix, YoVariableRegistry registry)
   {
      kdParallelMaxReductionRatio = new YoDouble("kdParallelMaxReductionRatio" + suffix, registry);
      dampingParallelToMotionDeadband = new YoDouble("parallelDampingDeadband" + suffix, registry);
      positionErrorForMinimumKd = new YoDouble("maxParallelDampingError" + suffix, registry);

      reset();
   }

   public void reset()
   {
      kdParallelMaxReductionRatio.set(1.0);
      dampingParallelToMotionDeadband.set(Double.POSITIVE_INFINITY);
      dampingParallelToMotionDeadband.set(Double.POSITIVE_INFINITY);
   }

   public void set(TangentialDampingGains tangentialDampingGains)
   {
      if (tangentialDampingGains != null)
      {
         set(tangentialDampingGains.getKdReductionRatio(), tangentialDampingGains.getParallelDampingDeadband(), tangentialDampingGains.getPositionErrorForMinimumKd());
      }
   }

   /** {@inheritDoc} */
   public void set(double kdReductionRatio, double parallelDampingDeadband, double positionErrorForMinimumKd)
   {
      setKdReductionRatio(kdReductionRatio);
      setParallelDampingDeadband(parallelDampingDeadband);
      setPositionErrorForMinimumKd(positionErrorForMinimumKd);
   }

   public void setKdReductionRatio(double kdReductionRatio)
   {
      kdParallelMaxReductionRatio.set(kdReductionRatio);
   }

   public void setParallelDampingDeadband(double parallelDampingDeadband)
   {
      dampingParallelToMotionDeadband.set(parallelDampingDeadband);
   }

   public void setPositionErrorForMinimumKd(double positionErrorForMinimumKd)
   {
      this.positionErrorForMinimumKd.set(positionErrorForMinimumKd);
   }

   /** {@inheritDoc} */
   public double getKdReductionRatio()
   {
      return kdParallelMaxReductionRatio.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getParallelDampingDeadband()
   {
      return dampingParallelToMotionDeadband.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getPositionErrorForMinimumKd()
   {
      return positionErrorForMinimumKd.getDoubleValue();
   }
}
