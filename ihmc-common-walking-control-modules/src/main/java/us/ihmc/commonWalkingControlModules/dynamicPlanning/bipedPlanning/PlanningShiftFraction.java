package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.robotics.saveableModule.SaveableModuleState;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PlanningShiftFraction extends SaveableModuleState
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
      registerDoubleToSave(swingDurationShiftFraction);
      registerDoubleToSave(swingSplitFraction);
      registerDoubleToSave(transferSplitFraction);
      registerDoubleToSave(transferWeightDistribution);

      clear();
   }

   public void clear()
   {
      swingDurationShiftFraction.setToNaN();
      swingSplitFraction.setToNaN();
      transferSplitFraction.setToNaN();
      transferWeightDistribution.setToNaN();
   }
}
