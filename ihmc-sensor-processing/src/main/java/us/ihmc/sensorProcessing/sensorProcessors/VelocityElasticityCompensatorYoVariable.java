package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;

public class VelocityElasticityCompensatorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final YoDouble stiffness;
   private final YoDouble rawJointVelocity;
   private final YoDouble jointTau;
   private final YoDouble jointDeflection;
   private final YoDouble jointDeflectionPrevious;
   private final YoDouble jointDeflectionDot;
   private final YoDouble maximumDeflection;

   private final double updateDT;

   public VelocityElasticityCompensatorYoVariable(String name, double updateDT, YoVariableRegistry registry)
   {
      this(name, null, null, null, null, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, YoDouble stiffness, double updateDT, YoVariableRegistry registry)
   {
      this(name, stiffness, null, null, null, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, YoDouble rawJointPosition, YoDouble jointTau, double updateDT,
         YoVariableRegistry registry)
   {
      this(name, null, null, rawJointPosition, jointTau, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, YoDouble stiffness, YoDouble rawJointPosition, YoDouble jointTau,
         double updateDT, YoVariableRegistry registry)
   {
      this(name, stiffness, null, rawJointPosition, jointTau, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, YoDouble stiffness, YoDouble maximumDeflection,
         YoDouble rawJointVelocity, YoDouble jointTau, double updateDT, YoVariableRegistry registry)
   {
      super(name, registry);

      this.updateDT = updateDT;

      if (maximumDeflection == null)
      {
         maximumDeflection = new YoDouble(name + "MaxDeflection", registry);
         maximumDeflection.set(0.1);
      }
      this.maximumDeflection = maximumDeflection;
      
      if (stiffness == null)
         stiffness = new YoDouble(name + "Stiffness", registry);
      this.stiffness = stiffness;

      this.rawJointVelocity = rawJointVelocity;
      this.jointTau = jointTau;
      jointDeflection = new YoDouble(name + "Deflection", registry);
      jointDeflectionPrevious = new YoDouble(name + "DeflectionPrevious", registry);
      jointDeflectionDot = new YoDouble(name + "DeflectionDot", registry);
   }

   public void setStiffness(double newStiffness)
   {
      stiffness.set(newStiffness);
   }

   public void setMaximuDeflection(double newMaxDeflection)
   {
      maximumDeflection.set(newMaxDeflection);
   }

   public void update()
   {
      if (rawJointVelocity == null || jointTau == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with non null "
                                        + "rawJointVelocity and jointTau variables to call update(), otherwise use update(double, double)");
      }

      update(rawJointVelocity.getDoubleValue(), jointTau.getDoubleValue());
   }

   public void update(double rawJointVelocity, double jointTau)
   {
      if (stiffness.getDoubleValue() < 1e-10)
         throw new RuntimeException("Joint stiffness is zero or negative!");

      double deflection = jointTau / stiffness.getDoubleValue();
      deflection = MathTools.clamp(deflection, maximumDeflection.getDoubleValue());
      jointDeflection.set(deflection);

      double newDeflectionDot = (jointDeflection.getDoubleValue() - jointDeflectionPrevious.getDoubleValue()) / updateDT;
      jointDeflectionDot.set(newDeflectionDot);

      jointDeflectionPrevious.set(jointDeflection.getDoubleValue());
      set(rawJointVelocity - jointDeflectionDot.getDoubleValue());
   }
}
