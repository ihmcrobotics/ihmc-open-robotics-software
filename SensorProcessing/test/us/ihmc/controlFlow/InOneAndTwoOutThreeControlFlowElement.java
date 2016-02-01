package us.ihmc.controlFlow;


public class InOneAndTwoOutThreeControlFlowElement extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<DataTypeOne> dataTypeOneInputPort = createInputPort();
   private final ControlFlowInputPort<DataTypeTwo> dataTypeTwoInputPort = createInputPort();
   
   private final ControlFlowOutputPort<DataTypeThree> dataTypeThreeOutputPort = createOutputPort();

   public ControlFlowInputPort<DataTypeOne> getDataTypeOneInputPort()
   {
      return dataTypeOneInputPort;
   }

   public ControlFlowInputPort<DataTypeTwo> getDataTypeTwoInputPort()
   {
      return dataTypeTwoInputPort;
   }

   public ControlFlowOutputPort<DataTypeThree> getDataTypeThreeOutputPort()
   {
      return dataTypeThreeOutputPort;
   }

   public void startComputation()
   {
      DataTypeOne dataTypeOne = dataTypeOneInputPort.getData();
      DataTypeTwo dataTypeTwo = dataTypeTwoInputPort.getData();

      double x = dataTypeOne.getX();
      double y = dataTypeOne.getY();
      
      double z = dataTypeTwo.getZ();
      
      double q = (x + y) * z;
      
      DataTypeThree dataTypeThree = new DataTypeThree();

      dataTypeThree.setQ(q);
      
      dataTypeThreeOutputPort.setData(dataTypeThree);
   }

   public void waitUntilComputationIsDone()
   {      
   }

   public void initialize()
   {
//    empty
   }
}
