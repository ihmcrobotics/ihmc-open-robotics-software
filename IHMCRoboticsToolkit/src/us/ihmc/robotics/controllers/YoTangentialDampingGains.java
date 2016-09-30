package us.ihmc.robotics.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class YoTangentialDampingGains implements TangentialDampingGains
{
   private final DoubleYoVariable kdParallelMaxReductionRatio;
   private final DoubleYoVariable dampingParallelToMotionDeadband;

   public YoTangentialDampingGains(String suffix, YoVariableRegistry registry)
   {
      kdParallelMaxReductionRatio = new DoubleYoVariable("kdParallelMaxReductionRatio" + suffix, registry);
      dampingParallelToMotionDeadband = new DoubleYoVariable("parallelDampingDeadband" + suffix, registry);

      reset();
   }

   public void reset()
   {
      kdParallelMaxReductionRatio.set(1.0);
      dampingParallelToMotionDeadband.set(Double.POSITIVE_INFINITY);
   }

   public void set(TangentialDampingGains tangentialDampingGains)
   {
      set(tangentialDampingGains.getKdReductionRatio(), tangentialDampingGains.getParallelDampingDeadband());
   }

   public void set(double kdReductionRatio, double parallelDampingDeadband)
   {
      setKdReductionRatio(kdReductionRatio);
      setParallelDampingDeadband(parallelDampingDeadband);
   }

   public void setKdReductionRatio(double kdReductionRatio)
   {
      kdParallelMaxReductionRatio.set(kdReductionRatio);
   }

   public void setParallelDampingDeadband(double parallelDampingDeadband)
   {
      dampingParallelToMotionDeadband.set(parallelDampingDeadband);
   }

   public DoubleYoVariable getYoKdReductionRatio()
   {
      return kdParallelMaxReductionRatio;
   }

   public DoubleYoVariable getYoParallelDampingDeadband()
   {
      return dampingParallelToMotionDeadband;
   }

   public double getKdReductionRatio()
   {
      return kdParallelMaxReductionRatio.getDoubleValue();
   }

   public double getParallelDampingDeadband()
   {
      return dampingParallelToMotionDeadband.getDoubleValue();
   }
}
