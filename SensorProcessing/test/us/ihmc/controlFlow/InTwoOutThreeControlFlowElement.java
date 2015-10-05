package us.ihmc.controlFlow;


public class InTwoOutThreeControlFlowElement extends AbstractControlFlowElement
{
   private ControlFlowInputPort<DataTypeTwo> inputPort = createInputPort();
   private ControlFlowOutputPort<DataTypeThree> outputPort = createOutputPort();
   
   public ControlFlowInputPort<DataTypeTwo> getInputPort()
   {
      return inputPort;
   }
   
   public ControlFlowOutputPort<DataTypeThree> getOutputPort()
   {
      return outputPort;
   }
   
   
   public void startComputation()
   {
      DataTypeTwo input = inputPort.getData();
      
      double z = input.getZ();
      
      DataTypeThree output = new DataTypeThree();
      output.setQ(z*z);   
      
      outputPort.setData(output);
   }
   
   public void waitUntilComputationIsDone()
   {
   }

   public String toString()
   {
      return "InTwoOutThreeControlFlowElement";
   }

   public void initialize()
   {
//    empty
   }
}

