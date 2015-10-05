package us.ihmc.controlFlow;

public class ControlFlowInputToInputPortConnector<DataType>
{
   private final ControlFlowInputPort<DataType> fromInputPort;
   private final ControlFlowInputPort<DataType> toInputPort;
   
   public ControlFlowInputToInputPortConnector(ControlFlowInputPort<DataType> fromInputPort, ControlFlowInputPort<DataType> toInputPort)
   {
      this.fromInputPort = fromInputPort;
      this.toInputPort = toInputPort;
   }
   
   public void sendDataAlongConnector()
   {
      toInputPort.setData(fromInputPort.getData());
   }

   public ControlFlowInputPort<DataType> getFromInputPort()
   {
      return fromInputPort;
   }
   
   public ControlFlowInputPort<DataType> getToInputPort()
   {
      return toInputPort;
   }
}

