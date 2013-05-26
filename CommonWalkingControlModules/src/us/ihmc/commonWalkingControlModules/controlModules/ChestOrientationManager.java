package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;


public class ChestOrientationManager
{
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final MomentumBasedController momentumBasedController;


   public ChestOrientationManager(MomentumBasedController momentumBasedController, ChestOrientationControlModule chestOrientationControlModule)
   {
      this.momentumBasedController = momentumBasedController;
      this.chestOrientationControlModule = chestOrientationControlModule;
   }

   public void compute()
   { 
      if (chestOrientationControlModule == null)
         return;

      chestOrientationControlModule.compute();
      momentumBasedController.setDesiredSpatialAcceleration(chestOrientationControlModule.getJacobian(),
            chestOrientationControlModule.getTaskspaceConstraintData());

   }
}
