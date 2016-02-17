package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

public class MomentumRateCommand extends InverseDynamicsCommand<MomentumRateCommand>
{
   private MomentumRateData momentumRateData;

   public MomentumRateCommand()
   {
      super(InverseDynamicsCommandType.MOMENTUM_RATE);
   }

   public MomentumRateCommand(MomentumRateData momentumRateOfChangeData)
   {
      this();
      this.momentumRateData = momentumRateOfChangeData;
   }

   public MomentumRateCommand(MomentumRateCommand momentumRateCommand)
   {
      this();
      set(momentumRateCommand);
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

   @Override
   public void set(MomentumRateCommand other)
   {
      this.momentumRateData = other.momentumRateData;
   }
}
