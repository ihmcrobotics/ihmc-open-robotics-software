package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.saveableModule.SaveableModuleState;
import us.ihmc.robotics.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoPTrajectoryGeneratorState extends SaveableModuleState
{
   private final RecyclingArrayList<PlanningFootstep> footsteps;
   private final RecyclingArrayList<PlanningTiming> footstepTimings;
   private final RecyclingArrayList<PlanningShiftFraction> footstepShiftFractions;

   private final YoFramePoint2D initialCoP;

   public CoPTrajectoryGeneratorState(YoRegistry registry)
   {
      footsteps = new RecyclingArrayList<>(3, () -> createFootstep(registry));
      footstepTimings = new RecyclingArrayList<>(3, () -> createTiming(registry));
      footstepShiftFractions = new RecyclingArrayList<>(3, () -> createShiftFractions(registry));

      initialCoP = new YoFramePoint2D("initialCoP", ReferenceFrame.getWorldFrame(), registry);
      SaveableModuleStateTools.registerYoTuple2DToSave(initialCoP, this);
   }

   public void clear()
   {
      for (int i = 0; i < footsteps.size(); i++)
         footsteps.get(i).clear();
      for (int i = 0; i < footstepTimings.size(); i++)
         footstepTimings.get(i).clear();
      for (int i = 0; i < footstepShiftFractions.size(); i++)
         footstepShiftFractions.get(i).clear();
      footsteps.clear();
      footstepTimings.clear();
      footstepShiftFractions.clear();
      initialCoP.setToNaN();
   }

   private PlanningFootstep createFootstep(YoRegistry registry)
   {
      PlanningFootstep footstep = new PlanningFootstep("" + footsteps.size(), registry);
      registerStateToSave(footstep);
      return footstep;
   }

   private PlanningTiming createTiming(YoRegistry registry)
   {
      PlanningTiming timing = new PlanningTiming("" + footstepTimings.size(), registry);
      registerStateToSave(timing);
      return timing;
   }

   private PlanningShiftFraction createShiftFractions(YoRegistry registry)
   {
      PlanningShiftFraction shiftFractions = new PlanningShiftFraction("" + footstepShiftFractions.size(), registry);
      registerStateToSave(shiftFractions);
      return shiftFractions;
   }
}
