package us.ihmc.controlFlow;

public class ControlFlowOutputToInputPortConnector<DataType>
{
   private final ControlFlowOutputPort<DataType> outputPort;
   private final ControlFlowInputPort<DataType> inputPort;
   
   public ControlFlowOutputToInputPortConnector(ControlFlowOutputPort<DataType> outputPort, ControlFlowInputPort<DataType> inputPort)
   {
      this.outputPort = outputPort;
      this.inputPort = inputPort;
   }
   
   public void sendDataAlongConnector()
   {
      inputPort.setData(outputPort.getData());
   }

   public ControlFlowOutputPort<DataType> getOutputPort()
   {
      return outputPort;
   }
   
   public ControlFlowInputPort<DataType> getInputPort()
   {
      return inputPort;
   }
}
