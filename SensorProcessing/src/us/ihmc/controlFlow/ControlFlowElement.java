package us.ihmc.controlFlow;

import java.util.List;

public interface ControlFlowElement
{   
   public abstract void initialize();
   public abstract void startComputation();
   public abstract void waitUntilComputationIsDone();
   public abstract List<ControlFlowInputPort<?>> getInputPorts();
   public abstract List<ControlFlowOutputPort<?>> getOutputPorts();
}
