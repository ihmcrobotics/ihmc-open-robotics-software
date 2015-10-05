package us.ihmc.controlFlow;


public class InTwoAndThreeOutTwoAndThreeControlFlowElement extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<DataTypeTwo> dataTypeTwoInputPort = createInputPort();
   private final ControlFlowInputPort<DataTypeThree> dataTypeThreeInputPort = createInputPort();

   private final ControlFlowOutputPort<DataTypeTwo> dataTypeTwoOutputPort = createOutputPort();
   private final ControlFlowOutputPort<DataTypeThree> dataTypeThreeOutputPort = createOutputPort();

   public ControlFlowInputPort<DataTypeTwo> getDataTypeTwoInputPort()
   {
      return dataTypeTwoInputPort;
   }

   public ControlFlowInputPort<DataTypeThree> getDataTypeThreeInputPort()
   {
      return dataTypeThreeInputPort;
   }

   public ControlFlowOutputPort<DataTypeTwo> getDataTypeTwoOutputPort()
   {
      return dataTypeTwoOutputPort;
   }

   public ControlFlowOutputPort<DataTypeThree> getDataTypeThreeOutputPort()
   {
      return dataTypeThreeOutputPort;
   }

   public void startComputation()
   {
      DataTypeTwo dataTypeTwoIn = dataTypeTwoInputPort.getData();
      DataTypeThree dataTypeThreeIn = dataTypeThreeInputPort.getData();

      double z = dataTypeTwoIn.getZ();
      double q = dataTypeThreeIn.getQ();

      DataTypeTwo dataTypeTwoOut = new DataTypeTwo();
      DataTypeThree dataTypeThreeOut = new DataTypeThree();

      dataTypeTwoOut.setZ(z * 2.0);
      dataTypeThreeOut.setQ(q * 3.0);

      dataTypeTwoOutputPort.setData(dataTypeTwoOut);
      dataTypeThreeOutputPort.setData(dataTypeThreeOut);
   }

   public void waitUntilComputationIsDone()
   {
   }

   public void initialize()
   {
//    empty
   }
}
