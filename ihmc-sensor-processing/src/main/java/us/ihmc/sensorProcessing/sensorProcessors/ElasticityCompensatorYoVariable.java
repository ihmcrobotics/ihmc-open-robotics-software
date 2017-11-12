package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;

public class ElasticityCompensatorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final YoDouble stiffness;
   private final YoDouble rawJointPosition;
   private final YoDouble jointTau;
   private final YoDouble maximumDeflection;

   public ElasticityCompensatorYoVariable(String name, YoVariableRegistry registry)
   {
      this(name, null, null, null, null, registry);
   }

   public ElasticityCompensatorYoVariable(String name, YoDouble stiffness, YoVariableRegistry registry)
   {
      this(name, stiffness, null, null, null, registry);
   }

   public ElasticityCompensatorYoVariable(String name, YoDouble rawJointPosition, YoDouble jointTau, YoVariableRegistry registry)
   {
      this(name, null, null, rawJointPosition, jointTau, registry);
   }

   public ElasticityCompensatorYoVariable(String name, YoDouble stiffness, YoDouble rawJointPosition, YoDouble jointTau, YoVariableRegistry registry)
   {
      this(name, stiffness, null, rawJointPosition, jointTau, registry);
   }

   public ElasticityCompensatorYoVariable(String name, YoDouble stiffness, YoDouble maximumDeflection, YoDouble rawJointPosition, YoDouble jointTau, YoVariableRegistry registry)
   {
      super(name, registry);

      if (maximumDeflection == null)
      {
         maximumDeflection = new YoDouble(name + "MaxDeflection", registry);
         maximumDeflection.set(0.1);
      }
      this.maximumDeflection = maximumDeflection;
      
      if (stiffness == null)
         stiffness = new YoDouble(name + "Stiffness", registry);
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
