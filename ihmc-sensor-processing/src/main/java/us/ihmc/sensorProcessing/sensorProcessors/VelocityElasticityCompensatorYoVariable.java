package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class VelocityElasticityCompensatorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final DoubleProvider stiffness;
   private final DoubleProvider rawJointVelocity;
   private final DoubleProvider jointTau;
   private final YoDouble jointDeflection;
   private final YoDouble jointDeflectionPrevious;
   private final YoDouble jointDeflectionDot;
   private final DoubleProvider maximumDeflection;

   private final double updateDT;

   public VelocityElasticityCompensatorYoVariable(String name, YoDouble stiffness, DoubleProvider maximumDeflection, double updateDT,
                                                  YoVariableRegistry registry)
   {
      this(name, stiffness, maximumDeflection, null, null, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, DoubleProvider stiffness, DoubleProvider maximumDeflection, DoubleProvider rawJointVelocity,
                                                  DoubleProvider jointTau, double updateDT, YoVariableRegistry registry)
   {
      super(name, registry);

      this.updateDT = updateDT;
      this.maximumDeflection = maximumDeflection;
      this.stiffness = stiffness;
      this.rawJointVelocity = rawJointVelocity;
      this.jointTau = jointTau;

      jointDeflection = new YoDouble(name + "Deflection", registry);
      jointDeflectionPrevious = new YoDouble(name + "DeflectionPrevious", registry);
      jointDeflectionDot = new YoDouble(name + "DeflectionDot", registry);
   }

   @Override
   public void update()
   {
      if (rawJointVelocity == null || jointTau == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with non null "
                                        + "rawJointVelocity and jointTau variables to call update(), otherwise use update(double, double)");
      }

      update(rawJointVelocity.getValue(), jointTau.getValue());
   }

   public void update(double rawJointVelocity, double jointTau)
   {
      if (stiffness.getValue() < 1e-10)
         throw new RuntimeException("Joint stiffness is zero or negative!");

      double deflection = jointTau / stiffness.getValue();
      deflection = MathTools.clamp(deflection, maximumDeflection.getValue());
      jointDeflection.set(deflection);

      double newDeflectionDot = (jointDeflection.getDoubleValue() - jointDeflectionPrevious.getDoubleValue()) / updateDT;
      jointDeflectionDot.set(newDeflectionDot);

      jointDeflectionPrevious.set(jointDeflection.getDoubleValue());
      set(rawJointVelocity - jointDeflectionDot.getDoubleValue());
   }

   @Override
   public void reset()
   {
      jointDeflectionPrevious.set(0.0);
   }
}
