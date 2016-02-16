package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

public class MomentumRateCommand extends InverseDynamicsCommand
{
   private MomentumRateData momentumRateData;

   public MomentumRateCommand(MomentumRateData momentumRateOfChangeData)
   {
      this.momentumRateData = momentumRateOfChangeData;
   }

   public MomentumRateCommand(MomentumRateCommand desiredRateOfChangeOfMomentumCommand)
   {
      this.momentumRateData = desiredRateOfChangeOfMomentumCommand.momentumRateData;
   }

   public MomentumRateData getMomentumRateData()
   {
      return momentumRateData;
   }

   public String toString()
   {
      return getClass().getSimpleName() + ": MomentumSubspace = " + momentumRateData.getMomentumSubspace();
   }

   public void setMomentumRateOfChangeData(MomentumRateData momentumRateOfChangeData)
   {
      this.momentumRateData = momentumRateOfChangeData;
   }
}
