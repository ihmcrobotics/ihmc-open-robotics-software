package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;

public interface MomentumRateOfChangeControlModule
{
   public abstract void initialize();
   public abstract void compute();
   public abstract void getMomentumRateOfChange(MomentumRateOfChangeData momentumRateOfChangeDataToPack);
}
