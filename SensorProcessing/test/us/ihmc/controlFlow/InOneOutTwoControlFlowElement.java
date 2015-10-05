package us.ihmc.controlFlow;

public class InOneOutTwoControlFlowElement extends AbstractControlFlowElement
{
   private ControlFlowInputPort<DataTypeOne> inputPort = createInputPort();
   private ControlFlowOutputPort<DataTypeTwo> outputPort = createOutputPort();
   
   public ControlFlowInputPort<DataTypeOne> getInputPort()
   {
      return inputPort;
   }
   
   public ControlFlowOutputPort<DataTypeTwo> getOutputPort()
   {
      return outputPort;
   }

   public void startComputation()
   {
      DataTypeOne input = inputPort.getData();
      
      double x = input.getX();
      double y = input.getY();
      
      DataTypeTwo output = new DataTypeTwo();

      output.setZ(x+y);   
      
      outputPort.setData(output);
   }

   public void waitUntilComputationIsDone()
   {
   }

   public String toString()
   {
      return "InOneOutTwoControlFlowElement\n";
   }

   public void initialize()
   {
//    empty
   }
}
