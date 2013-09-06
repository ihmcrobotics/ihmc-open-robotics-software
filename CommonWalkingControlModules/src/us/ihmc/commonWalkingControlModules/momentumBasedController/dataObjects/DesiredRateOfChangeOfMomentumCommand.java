package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;


public class DesiredRateOfChangeOfMomentumCommand
{
   private final MomentumRateOfChangeData momentumRateOfChangeData;

   public DesiredRateOfChangeOfMomentumCommand(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      this.momentumRateOfChangeData = momentumRateOfChangeData;
   }
   
   

   public MomentumRateOfChangeData getMomentumRateOfChangeData()
   {
      return momentumRateOfChangeData;
   }



   public String toString()
   {
      return "DesiredRateOfChangeOfMomentumCommand: MomentumSubspace = "
            + momentumRateOfChangeData.getMomentumSubspace();
   }
}

