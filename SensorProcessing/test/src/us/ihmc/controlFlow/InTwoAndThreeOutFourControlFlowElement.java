package us.ihmc.controlFlow;

import us.ihmc.euclid.tuple3D.Point3D;

public class InTwoAndThreeOutFourControlFlowElement extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<DataTypeTwo> dataTypeTwoInputPort = createInputPort();
   private final ControlFlowInputPort<DataTypeThree> dataTypeThreeInputPort = createInputPort();
   
   private final ControlFlowOutputPort<DataTypeFour> dataTypeFourOutputPort = createOutputPort();
 
   
   public ControlFlowInputPort<DataTypeTwo> getDataTypeTwoInputPort()
   {
      return dataTypeTwoInputPort;
   }

   public ControlFlowInputPort<DataTypeThree> getDataTypeThreeInputPort()
   {
      return dataTypeThreeInputPort;
   }

   public ControlFlowOutputPort<DataTypeFour> getDataTypeFourOutputPort()
   {
      return dataTypeFourOutputPort;
   }

   public void startComputation()
   {
      DataTypeTwo dataTypeTwo = dataTypeTwoInputPort.getData();
      DataTypeThree dataTypeThree = dataTypeThreeInputPort.getData();
     
      double z = dataTypeTwo.getZ();
      double q = dataTypeThree.getQ();
      
      DataTypeFour dataTypeFour = new DataTypeFour();

      dataTypeFour.setPoint(new Point3D(z, q, z+q));
      
      dataTypeFourOutputPort.setData(dataTypeFour);
   }

   public void waitUntilComputationIsDone()
   {      
   }

   public String toString()
   {
      return "InTwoAndThreeOutFourControlFlowElement";
   }

   public void initialize()
   {
//    empty
   }
}
