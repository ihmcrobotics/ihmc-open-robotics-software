package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;

public class NullMomentumRateOfChangeControlModule implements MomentumRateOfChangeControlModule
{
   private final MomentumRateOfChangeData momentumRateOfChangeData;

   public NullMomentumRateOfChangeControlModule()
   {
      momentumRateOfChangeData = new MomentumRateOfChangeData(null);
      momentumRateOfChangeData.setEmpty();
   }

   @Override
   public void initialize()
   {
      // empty
   }

   @Override
   public void compute()
   {
      // empty
   }

   @Override
   public void getMomentumRateOfChange(MomentumRateOfChangeData momentumRateOfChangeDataToPack)
   {
      momentumRateOfChangeDataToPack.set(momentumRateOfChangeData);
   }
}
