package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.tools.saveableModule.SaveableModuleState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingCoPTrajectoryParameters extends SaveableModuleState
{
   private static final double defaultFractionSupportForShift = 0.2;


   private final DoubleParameter fractionSupportForShift;


   public JumpingCoPTrajectoryParameters(YoRegistry registry)
   {
      fractionSupportForShift = new DoubleParameter("fractionSupportForShift", registry, defaultFractionSupportForShift);

      registerVariableToSave(fractionSupportForShift);
   }

   public double getFractionSupportForShift()
   {
      return fractionSupportForShift.getValue();
   }

}
