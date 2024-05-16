package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Map;

public class DiagnosticPostureAdjustmentCalculator implements WholeBodyPostureAdjustmentProvider
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoFunctionGeneratorNew[] functionGenerators;
   private final Map<String, YoFunctionGeneratorNew> jointNameToGenerator = new HashMap<>();

   public DiagnosticPostureAdjustmentCalculator(OneDoFJointBasics[] oneDoFJoints, double controlDT, YoRegistry parentRegistry)
   {
      functionGenerators = new YoFunctionGeneratorNew[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         functionGenerators[i] = new YoFunctionGeneratorNew("diagnostic_posture_" + oneDoFJoints[i].getName(), controlDT, registry);
         jointNameToGenerator.put(oneDoFJoints[i].getName(), functionGenerators[i]);
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      for (int i = 0; i < functionGenerators.length; i++)
      {
         functionGenerators[i].update();
      }
   }

   @Override
   public boolean isEnabled()
   {
      return true;
   }

   @Override
   public double getDesiredJointPositionOffset(String jointName)
   {
      return jointNameToGenerator.get(jointName).getValue();
   }

   @Override
   public double getDesiredJointVelocityOffset(String jointName)
   {
      return jointNameToGenerator.get(jointName).getValueDot();
   }

   @Override
   public double getDesiredJointAccelerationOffset(String jointName)
   {
      return jointNameToGenerator.get(jointName).getValueDot();
   }
}
