package us.ihmc.controlFlow;

import us.ihmc.euclid.tuple3D.Point3D;

public class InFourAndFiveOutFourControlFlowElement extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<DataTypeFour> dataTypeFourInputPort = createInputPort();
   private final ControlFlowInputPort<DataTypeFive> dataTypeFiveInputPort = createInputPort();
   
   private final ControlFlowOutputPort<DataTypeFour> dataTypeFourOutputPort = createOutputPort();

   public ControlFlowInputPort<DataTypeFour> getDataTypeFourInputPort()
   {
      return dataTypeFourInputPort;
   }

   public ControlFlowInputPort<DataTypeFive> getDataTypeFiveInputPort()
   {
      return dataTypeFiveInputPort;
   }

   public ControlFlowOutputPort<DataTypeFour> getDataTypeFourOutputPort()
   {
      return dataTypeFourOutputPort;
   }

   public void startComputation()
   {
      DataTypeFour dataTypeFourIn = dataTypeFourInputPort.getData();
      DataTypeFive dataTypeFiveIn = dataTypeFiveInputPort.getData();
      
      Point3D point = dataTypeFourIn.getPoint();
      String string = dataTypeFiveIn.getString();
      
      DataTypeFour dataTypeFourOut = new DataTypeFour();

      dataTypeFourOut.setPoint(point);
      
      if (string.equals("scale"))
      {
         dataTypeFourOut.scale(2.0);
      }
      
      dataTypeFourOutputPort.setData(dataTypeFourOut);
   }

   public void waitUntilComputationIsDone()
   {      
   }

   public void initialize()
   {
//    empty
   }
}
