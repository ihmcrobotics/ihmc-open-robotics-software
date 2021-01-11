package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PlanningShiftFraction extends YoSaveableModuleState
{
   /** The nominal fraction of the swing duration of a footstep as specified in the FootstepData that is spent shifting the CoP from the heel to the toe.*/
   private final YoDouble swingDurationShiftFraction;

   /** The nominal fraction of the swing duration of a footstep spent shifting from the heel to the toe as specified in the FootstepData that is spent shifting
    *  the CoP from the heel to the ball.*/
   private final YoDouble swingSplitFraction;

   /** The nominal fraction of the transfer duration of a footstep as specified in the FootstepData that is spent shifting the CoP to the midpoint.*/
   private final YoDouble transferSplitFraction;

   /** The weight distribution of the CoP midpoint for the transfer duration between the trailing foot (0.0) and leading foot (1.0) */
   private final YoDouble transferWeightDistribution;

   public PlanningShiftFraction(String suffix, YoRegistry registry)
   {
      swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + suffix, registry);
      swingSplitFraction = new YoDouble("swingSplitFraction" + suffix, registry);
      transferSplitFraction = new YoDouble("transferSplitFraction" + suffix, registry);
      transferWeightDistribution = new YoDouble("transferWeightDistribution" + suffix, registry);
      registerVariableToSave(swingDurationShiftFraction);
      registerVariableToSave(swingSplitFraction);
      registerVariableToSave(transferSplitFraction);
      registerVariableToSave(transferWeightDistribution);

      clear();
   }

   public double getSwingDurationShiftFraction()
   {
      return swingDurationShiftFraction.getDoubleValue();
   }

   public double getSwingSplitFraction()
   {
      return swingSplitFraction.getDoubleValue();
   }

   public double getTransferSplitFraction()
   {
      return transferSplitFraction.getDoubleValue();
   }

   public double getTransferWeightDistribution()
   {
      return transferWeightDistribution.getDoubleValue();
   }

   public void clear()
   {
      swingDurationShiftFraction.setToNaN();
      swingSplitFraction.setToNaN();
      transferSplitFraction.setToNaN();
      transferWeightDistribution.setToNaN();
   }

   public void set(FootstepShiftFractions shiftFractions)
   {
      swingDurationShiftFraction.set(shiftFractions.getSwingDurationShiftFraction());
      swingSplitFraction.set(shiftFractions.getSwingSplitFraction());
      transferSplitFraction.set(shiftFractions.getTransferSplitFraction());
      transferWeightDistribution.set(shiftFractions.getTransferWeightDistribution());
   }
}
