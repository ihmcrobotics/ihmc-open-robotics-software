package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.tools.saveableModule.SaveableModuleState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingCoPTrajectoryParameters extends SaveableModuleState
{
   private static final double defaultFractionSupportForShift = 0.2;
   private static final double defaultSupportDurationValue = 0.75;
   private static final double defaultFlightDurationValue = 0.2;
   private static final double defaultFootWidthValue = 0.2;

   private final DoubleParameter fractionSupportForShift;
   private final DoubleParameter defaultSupportDuration;
   private final DoubleParameter defaultFlightDuration;
   private final DoubleParameter defaultFootWidth;

   public JumpingCoPTrajectoryParameters(YoRegistry registry)
   {
      fractionSupportForShift = new DoubleParameter("fractionSupportForShift", registry, defaultFractionSupportForShift);
      defaultSupportDuration = new DoubleParameter("defaultSupportDuration", registry, defaultSupportDurationValue);
      defaultFlightDuration = new DoubleParameter("defaultFlightDuration", registry, defaultFlightDurationValue);
      defaultFootWidth = new DoubleParameter("defaultFootWidth", registry, defaultFootWidthValue);

      registerVariableToSave(fractionSupportForShift);
      registerVariableToSave(defaultSupportDuration);
      registerVariableToSave(defaultFlightDuration);
      registerVariableToSave(defaultFootWidth);
   }

   public double getFractionSupportForShift()
   {
      return fractionSupportForShift.getValue();
   }

   public double getDefaultSupportDuration()
   {
      return defaultSupportDuration.getValue();
   }

   public double getDefaultFlightDuration()
   {
      return defaultFlightDuration.getValue();
   }

   public double getDefaultFootWidth()
   {
      return defaultFootWidth.getValue();
   }
}
