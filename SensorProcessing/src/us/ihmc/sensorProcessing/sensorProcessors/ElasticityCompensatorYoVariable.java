package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;


public class ElasticityCompensatorYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final DoubleYoVariable stiffness;
   private final DoubleYoVariable rawJointPosition;
   private final DoubleYoVariable jointTau;
   private final DoubleYoVariable maximumDeflection;

   public ElasticityCompensatorYoVariable(String name, YoVariableRegistry registry)
   {
      this(name, null, null, null, null, registry);
   }

   public ElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, YoVariableRegistry registry)
   {
      this(name, stiffness, null, null, null, registry);
   }

   public ElasticityCompensatorYoVariable(String name, DoubleYoVariable rawJointPosition, DoubleYoVariable jointTau, YoVariableRegistry registry)
   {
      this(name, null, null, rawJointPosition, jointTau, registry);
   }

   public ElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, DoubleYoVariable rawJointPosition, DoubleYoVariable jointTau, YoVariableRegistry registry)
   {
      this(name, stiffness, null, rawJointPosition, jointTau, registry);
   }

   public ElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, DoubleYoVariable maximumDeflection, DoubleYoVariable rawJointPosition, DoubleYoVariable jointTau, YoVariableRegistry registry)
   {
      super(name, registry);

      if (maximumDeflection == null)
      {
         maximumDeflection = new DoubleYoVariable(name + "MaxDeflection", registry);
         maximumDeflection.set(0.1);
      }
      this.maximumDeflection = maximumDeflection;
      
      if (stiffness == null)
         stiffness = new DoubleYoVariable(name + "Stiffness", registry);
      this.stiffness = stiffness;

      this.rawJointPosition = rawJointPosition;
      this.jointTau = jointTau;
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
      if (rawJointPosition == null || jointTau == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with non null "
                                        + "rawJointPosition and jointTau variables to call update(), otherwise use update(double, double)");
      }

      update(rawJointPosition.getDoubleValue(), jointTau.getDoubleValue());
   }

   public void update(double rawJointPosition, double jointTau)
   {
      double jointDeflection;
      if (stiffness.getDoubleValue() > 1e-10)
      {
         jointDeflection = jointTau / stiffness.getDoubleValue();
         jointDeflection = MathTools.clamp(jointDeflection, maximumDeflection.getDoubleValue());
         this.set(rawJointPosition - jointDeflection);
      }
      else
      {
         throw new RuntimeException("Joint stiffness is zero or negative!");
      }
   }
}
