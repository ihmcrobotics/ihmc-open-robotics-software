package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;


public class DesiredRateOfChangeOfMomentumCommand
{
   private MomentumRateOfChangeData momentumRateOfChangeData;

   public DesiredRateOfChangeOfMomentumCommand(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      this.momentumRateOfChangeData = momentumRateOfChangeData;
   }

   public DesiredRateOfChangeOfMomentumCommand(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      this.momentumRateOfChangeData = desiredRateOfChangeOfMomentumCommand.momentumRateOfChangeData;
   }

   public MomentumRateOfChangeData getMomentumRateOfChangeData()
   {
      return momentumRateOfChangeData;
   }

   public String toString()
   {
      return "DesiredRateOfChangeOfMomentumCommand: MomentumSubspace = " + momentumRateOfChangeData.getMomentumSubspace();
   }

   public void setMomentumRateOfChangeData(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      this.momentumRateOfChangeData = momentumRateOfChangeData;
   }
}
