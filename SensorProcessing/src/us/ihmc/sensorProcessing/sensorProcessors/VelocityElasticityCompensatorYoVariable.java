package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;


public class VelocityElasticityCompensatorYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final DoubleYoVariable stiffness;
   private final DoubleYoVariable rawJointVelocity;
   private final DoubleYoVariable jointTau;
   private final DoubleYoVariable jointDeflection;
   private final DoubleYoVariable jointDeflectionPrevious;
   private final DoubleYoVariable jointDeflectionDot;
   private final DoubleYoVariable maximumDeflection;

   private final double updateDT;

   public VelocityElasticityCompensatorYoVariable(String name, double updateDT, YoVariableRegistry registry)
   {
      this(name, null, null, null, null, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, double updateDT, YoVariableRegistry registry)
   {
      this(name, stiffness, null, null, null, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, DoubleYoVariable rawJointPosition, DoubleYoVariable jointTau, double updateDT,
         YoVariableRegistry registry)
   {
      this(name, null, null, rawJointPosition, jointTau, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, DoubleYoVariable rawJointPosition, DoubleYoVariable jointTau,
         double updateDT, YoVariableRegistry registry)
   {
      this(name, stiffness, null, rawJointPosition, jointTau, updateDT, registry);
   }

   public VelocityElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, DoubleYoVariable maximumDeflection,
         DoubleYoVariable rawJointVelocity, DoubleYoVariable jointTau, double updateDT, YoVariableRegistry registry)
   {
      super(name, registry);

      this.updateDT = updateDT;

      if (maximumDeflection == null)
      {
         maximumDeflection = new DoubleYoVariable(name + "MaxDeflection", registry);
         maximumDeflection.set(0.1);
      }
      this.maximumDeflection = maximumDeflection;
      
      if (stiffness == null)
         stiffness = new DoubleYoVariable(name + "Stiffness", registry);
      this.stiffness = stiffness;

      this.rawJointVelocity = rawJointVelocity;
      this.jointTau = jointTau;
      jointDeflection = new DoubleYoVariable(name + "Deflection", registry);
      jointDeflectionPrevious = new DoubleYoVariable(name + "DeflectionPrevious", registry);
      jointDeflectionDot = new DoubleYoVariable(name + "DeflectionDot", registry);
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
