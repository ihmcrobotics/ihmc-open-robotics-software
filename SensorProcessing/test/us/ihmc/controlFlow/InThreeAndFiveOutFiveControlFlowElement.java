package us.ihmc.controlFlow;


public class InThreeAndFiveOutFiveControlFlowElement extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<DataTypeThree> dataTypeThreeInputPort = createInputPort();
   private final ControlFlowInputPort<DataTypeFive> dataTypeFiveInputPort = createInputPort();

   private final ControlFlowOutputPort<DataTypeFive> dataTypeFiveOutputPort = createOutputPort();

   public ControlFlowInputPort<DataTypeThree> getDataTypeThreeInputPort()
   {
      return dataTypeThreeInputPort;
   }

   public ControlFlowInputPort<DataTypeFive> getDataTypeFiveInputPort()
   {
      return dataTypeFiveInputPort;
   }

   public ControlFlowOutputPort<DataTypeFive> getDataTypeFiveOutputPort()
   {
      return dataTypeFiveOutputPort;
   }

   public void startComputation()
   {
      DataTypeThree dataTypeThreeIn = dataTypeThreeInputPort.getData();
      DataTypeFive dataTypeFiveIn = dataTypeFiveInputPort.getData();
      
      double q = dataTypeThreeIn.getQ();
      String string = dataTypeFiveIn.getString();
      
      DataTypeFive dataTypeFiveOut = new DataTypeFive();
      dataTypeFiveOut.setString(string + ", q = " + q);
      
      dataTypeFiveOutputPort.setData(dataTypeFiveOut);
   }

   public void waitUntilComputationIsDone()
   {      
   } 

   public void initialize()
   {
//    empty
   }
}
