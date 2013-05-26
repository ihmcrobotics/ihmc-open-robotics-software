package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;

public class HeadOrientationManager
{
   private final HeadOrientationControlModule headOrientationControlModule;
   private final MomentumBasedController momentumBasedController;


   public HeadOrientationManager(MomentumBasedController momentumBasedController, HeadOrientationControlModule headOrientationControlModule)
   {
      this.momentumBasedController = momentumBasedController;
      this.headOrientationControlModule = headOrientationControlModule;
   }

   public void compute()
   { 
      if (headOrientationControlModule != null)
      {
         headOrientationControlModule.compute();
         momentumBasedController.setDesiredSpatialAcceleration(headOrientationControlModule.getJacobian(),
               headOrientationControlModule.getTaskspaceConstraintData());
      }
   }
}
