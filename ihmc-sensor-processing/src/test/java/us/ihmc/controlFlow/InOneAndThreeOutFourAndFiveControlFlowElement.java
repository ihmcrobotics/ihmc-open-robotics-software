package us.ihmc.controlFlow;

import us.ihmc.euclid.tuple3D.Point3D;

public class InOneAndThreeOutFourAndFiveControlFlowElement extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<DataTypeOne> dataTypeOneInputPort = createInputPort();
   private final ControlFlowInputPort<DataTypeThree> dataTypeThreeInputPort = createInputPort();
   
   private final ControlFlowOutputPort<DataTypeFour> dataTypeFourOutputPort = createOutputPort();
   private final ControlFlowOutputPort<DataTypeFive> dataTypeFiveOutputPort = createOutputPort();
   
   public ControlFlowInputPort<DataTypeOne> getDataTypeOneInputPort()
   {
      return dataTypeOneInputPort;
   }

   public ControlFlowInputPort<DataTypeThree> getDataTypeThreeInputPort()
   {
      return dataTypeThreeInputPort;
   }

   public ControlFlowOutputPort<DataTypeFour> getDataTypeFourOutputPort()
   {
      return dataTypeFourOutputPort;
   }

   public ControlFlowOutputPort<DataTypeFive> getDataTypeFiveOutputPort()
   {
      return dataTypeFiveOutputPort;
   }
  
   public void startComputation()
   {
      DataTypeOne dataTypeOne = dataTypeOneInputPort.getData();
      DataTypeThree dataTypeThree = dataTypeThreeInputPort.getData();
      
      double x = dataTypeOne.getX();
      double y = dataTypeOne.getY();
      
      double q = dataTypeThree.getQ();
      
      DataTypeFour dataTypeFour = new DataTypeFour();
      DataTypeFive dataTypeFive = new DataTypeFive();
      
      dataTypeFour.setPoint(new Point3D(x, y, q));
      dataTypeFive.setString("scale");
      
      dataTypeFourOutputPort.setData(dataTypeFour);
      dataTypeFiveOutputPort.setData(dataTypeFive);
   }

   public void waitUntilComputationIsDone()
   {      
   }

   public void initialize()
   {
//    empty
   }
}
