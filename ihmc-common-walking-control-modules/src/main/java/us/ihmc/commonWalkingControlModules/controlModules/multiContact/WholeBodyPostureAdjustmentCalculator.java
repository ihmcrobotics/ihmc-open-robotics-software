package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Map;

public class WholeBodyPostureAdjustmentCalculator implements WholeBodyPostureAdjustmentProvider
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   /* Diagnostic tools */
   private final YoFunctionGeneratorNew[] functionGenerators;
   private final Map<String, YoFunctionGeneratorNew> jointNameToGenerator = new HashMap<>();

   public WholeBodyPostureAdjustmentCalculator(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;

      OneDoFJointBasics[] oneDoFJoints = controllerToolbox.getControlledOneDoFJoints();
      functionGenerators = new YoFunctionGeneratorNew[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         functionGenerators[i] = new YoFunctionGeneratorNew("mc_posture_" + oneDoFJoints[i].getName(), controllerToolbox.getControlDT(), registry);
         jointNameToGenerator.put(oneDoFJoints[i].getName(), functionGenerators[i]);
      }

      controllerToolbox.getYoVariableRegistry().addChild(registry);
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
      return 0;
   }

   @Override
   public double getDesiredJointVelocityOffset(String jointName)
   {
      return 0;
   }
}
