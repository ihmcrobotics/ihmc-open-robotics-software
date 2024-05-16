package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Map;

public class DiagnosticPostureAdjustmentCalculator implements WholeBodyPostureAdjustmentProvider
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoFunctionGeneratorNew[] oneDoFJointFunctionGenerators;
   private final YoFunctionGeneratorNew pelvisHeightFunctionGenerator;

   private final Map<String, YoFunctionGeneratorNew> oneDoFJointNameToGenerator = new HashMap<>();

   public DiagnosticPostureAdjustmentCalculator(OneDoFJointBasics[] oneDoFJoints, double controlDT, YoRegistry parentRegistry)
   {
      oneDoFJointFunctionGenerators = new YoFunctionGeneratorNew[oneDoFJoints.length];

      String prefix = "diagnostic_posture_";
      pelvisHeightFunctionGenerator = new YoFunctionGeneratorNew(prefix + "PelvisHeight", controlDT, registry);

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         oneDoFJointFunctionGenerators[i] = new YoFunctionGeneratorNew(prefix + oneDoFJoints[i].getName(), controlDT, registry);
         oneDoFJointNameToGenerator.put(oneDoFJoints[i].getName(), oneDoFJointFunctionGenerators[i]);
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      pelvisHeightFunctionGenerator.update();

      for (int i = 0; i < oneDoFJointFunctionGenerators.length; i++)
      {
         oneDoFJointFunctionGenerators[i].update();
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
      return oneDoFJointNameToGenerator.get(jointName).getValue();
   }

   @Override
   public double getDesiredJointVelocityOffset(String jointName)
   {
      return oneDoFJointNameToGenerator.get(jointName).getValueDot();
   }

   @Override
   public double getDesiredJointAccelerationOffset(String jointName)
   {
      return oneDoFJointNameToGenerator.get(jointName).getValueDDot();
   }

   @Override
   public double getFloatingBasePositionOffsetZ()
   {
      return pelvisHeightFunctionGenerator.getValue();
   }

   @Override
   public double getFloatingBaseVelocityOffsetZ()
   {
      return pelvisHeightFunctionGenerator.getValueDot();
   }

   @Override
   public double getFloatingBaseAccelerationOffsetZ()
   {
      return pelvisHeightFunctionGenerator.getValueDDot();
   }
}
