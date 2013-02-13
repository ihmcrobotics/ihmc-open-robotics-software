package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.controlFlow.ControlFlowOutputPort;

public class NullMomentumRateOfChangeControlModule implements MomentumRateOfChangeControlModule
{
   private final ControlFlowOutputPort<MomentumRateOfChangeData> momentumRateOfChangeOutputPort = new ControlFlowOutputPort<MomentumRateOfChangeData>(this);

   public NullMomentumRateOfChangeControlModule()
   {
      MomentumRateOfChangeData momentumRateOfChangeData = new MomentumRateOfChangeData(null);
      momentumRateOfChangeData.setEmpty();
      momentumRateOfChangeOutputPort.setData(momentumRateOfChangeData);
   }

   public void startComputation()
   {
      // empty
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<MomentumRateOfChangeData> getMomentumRateOfChangeOutputPort()
   {
      return momentumRateOfChangeOutputPort;
   }
}
