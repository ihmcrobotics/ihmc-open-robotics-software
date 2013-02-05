package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;

public interface MomentumRateOfChangeControlModule extends ControlFlowElement
{
   public abstract ControlFlowOutputPort<MomentumRateOfChangeData> getMomentumRateOfChangeOutputPort();
}
