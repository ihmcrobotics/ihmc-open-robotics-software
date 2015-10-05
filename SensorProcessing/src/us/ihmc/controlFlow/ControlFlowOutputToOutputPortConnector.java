package us.ihmc.controlFlow;


public class ControlFlowOutputToOutputPortConnector<DataType>
{
   private final ControlFlowOutputPort<DataType> fromOutputPort;
   private final ControlFlowOutputPort<DataType> toOutputPort;
   
   public ControlFlowOutputToOutputPortConnector(ControlFlowOutputPort<DataType> fromOutputPort, ControlFlowOutputPort<DataType> toOutputPort)
   {
      this.fromOutputPort = fromOutputPort;
      this.toOutputPort = toOutputPort;
   }
   
   public void sendDataAlongConnector()
   {
      toOutputPort.setData(fromOutputPort.getData());
   }

   public ControlFlowOutputPort<DataType> getFromOutputPort()
   {
      return fromOutputPort;
   }
   
   public ControlFlowOutputPort<DataType> getToOutputPort()
   {
      return toOutputPort;
   }
}


