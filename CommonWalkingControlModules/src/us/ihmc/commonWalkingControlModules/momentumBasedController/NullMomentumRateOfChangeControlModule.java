package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;

public class NullMomentumRateOfChangeControlModule extends AbstractControlFlowElement implements MomentumRateOfChangeControlModule
{
   private final ControlFlowOutputPort<MomentumRateOfChangeData> momentumRateOfChangeOutputPort = createOutputPort("momentumRateOfChangeOutputPort");

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

   public void initialize()
   {
//    empty
   }
}
