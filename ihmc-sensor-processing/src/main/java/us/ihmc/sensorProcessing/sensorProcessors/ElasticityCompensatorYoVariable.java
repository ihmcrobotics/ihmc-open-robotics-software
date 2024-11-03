package us.ihmc.sensorProcessing.sensorProcessors;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ElasticityCompensatorYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final DoubleProvider stiffness;
   private final DoubleProvider rawJointPosition;
   private final DoubleProvider jointTau;
   private final DoubleProvider maximumDeflection;

   public ElasticityCompensatorYoVariable(String name, DoubleProvider stiffness, DoubleProvider maximumDeflection, YoRegistry registry)
   {
      this(name, stiffness, maximumDeflection, null, null, registry);
   }

   public ElasticityCompensatorYoVariable(String name, DoubleProvider stiffness, DoubleProvider maximumDeflection, DoubleProvider rawJointPosition, DoubleProvider jointTau, YoRegistry registry)
   {
      super(name, registry);

      this.maximumDeflection = maximumDeflection;
      this.stiffness = stiffness;
      this.rawJointPosition = rawJointPosition;
      this.jointTau = jointTau;
   }

   @Override
   public void update()
   {
      if (rawJointPosition == null || jointTau == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with non null "
                                        + "rawJointPosition and jointTau variables to call update(), otherwise use update(double, double)");
      }

      update(rawJointPosition.getValue(), jointTau.getValue());
   }

   public void update(double rawJointPosition, double jointTau)
   {
      double jointDeflection;
      if (stiffness.getValue() > 1e-10)
      {
         jointDeflection = jointTau / stiffness.getValue();
         jointDeflection = MathTools.clamp(jointDeflection, maximumDeflection.getValue());
         this.set(rawJointPosition - jointDeflection);
      }
      else
      {
         throw new RuntimeException("Joint stiffness is zero or negative!");
      }
   }
}
