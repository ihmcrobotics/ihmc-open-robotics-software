package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingCoPTrajectoryParameters extends YoSaveableModuleState
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
