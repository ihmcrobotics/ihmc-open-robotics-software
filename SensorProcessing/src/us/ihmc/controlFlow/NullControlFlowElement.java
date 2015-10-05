package us.ihmc.controlFlow;

import java.util.List;

public class NullControlFlowElement implements ControlFlowElement
{
   public void startComputation()
   {
   }

   public void waitUntilComputationIsDone()
   {
   }

   public List<ControlFlowInputPort<?>> getInputPorts()
   {
      return null;
   }

   public List<ControlFlowOutputPort<?>> getOutputPorts()
   {
      return null;
   }

   public void initialize()
   {
//    empty
   }
}
