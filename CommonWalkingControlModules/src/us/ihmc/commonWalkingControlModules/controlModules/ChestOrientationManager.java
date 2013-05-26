package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;


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

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector desiredAngularAcceleration)
   {
      chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);    
   }
}
