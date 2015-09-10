package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.ProcessingYoVariable;


public class ElasticityCompensatorYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final DoubleYoVariable stiffness;
   private final DoubleYoVariable rawJointPosition;
   private final DoubleYoVariable jointTau;
   private final DoubleYoVariable maximuDeflection;

   public ElasticityCompensatorYoVariable(String name, YoVariableRegistry registry)
   {
      super(name, registry);
      
      maximuDeflection = new DoubleYoVariable(name + "MaxDeflection", registry);
      maximuDeflection.set(0.1);
      stiffness = new DoubleYoVariable(name + "Stiffness", registry);
      rawJointPosition = null;
      jointTau = null;
   }

   public ElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, YoVariableRegistry registry)
   {
      super(name, registry);

      maximuDeflection = new DoubleYoVariable(name + "MaxDeflection", registry);
      maximuDeflection.set(0.1);
      this.stiffness = stiffness;
      rawJointPosition = null;
      jointTau = null;
   }

   public ElasticityCompensatorYoVariable(String name, DoubleYoVariable rawJointPosition, DoubleYoVariable jointTau, YoVariableRegistry registry)
   {
      super(name, registry);

      maximuDeflection = new DoubleYoVariable(name + "MaxDeflection", registry);
      maximuDeflection.set(0.1);
      this.stiffness = new DoubleYoVariable(name + "Stiffness", registry);
      this.rawJointPosition = rawJointPosition;
      this.jointTau = jointTau;
   }

   public ElasticityCompensatorYoVariable(String name, DoubleYoVariable stiffness, DoubleYoVariable rawJointPosition, DoubleYoVariable jointTau, YoVariableRegistry registry)
   {
      super(name, registry);

      maximuDeflection = new DoubleYoVariable(name + "MaxDeflection", registry);
      maximuDeflection.set(0.1);
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
      maximuDeflection.set(newMaxDeflection);
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
         jointDeflection = MathTools.clipToMinMax(jointDeflection, maximuDeflection.getDoubleValue());
         this.set(rawJointPosition - jointDeflection);
      }
      else
      {
         throw new RuntimeException("Joint stiffness is zero or negative!");
      }
   }
}
