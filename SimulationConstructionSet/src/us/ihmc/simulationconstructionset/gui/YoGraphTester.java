package us.ihmc.simulationconstructionset.gui;

import javax.swing.JFrame;

import us.ihmc.yoVariables.dataBuffer.DataEntry;
import us.ihmc.yoVariables.dataBuffer.DataEntryHolder;
import us.ihmc.yoVariables.dataBuffer.TimeDataHolder;
import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;

public class YoGraphTester
{
   public void testYoGraph()
   {
      SelectedVariableHolder selectedVariableHolder = new SelectedVariableHolder();


      JFrame jFrame = new JFrame("testYoGraph");


      YoGraphRemover yoGraphRemover = new YoGraphRemover()
      {
         @Override
         public void removeGraph(YoGraph yoGraph)
         {
         }
      };

      DataEntryHolder dataEntryHolder = new DataEntryHolder()
      {
         @Override
         public DataEntry getEntry(YoVariable<?> yoVariable)
         {
            return null;
         }
      };

      TimeDataHolder timeDataHolder = new MinimalTimeDataHolder(200);
      

      GraphIndicesHolder graphIndicesHolder = new MinimalGraphIndicesHolder();

      YoGraph yoGraph = new YoGraph(graphIndicesHolder, yoGraphRemover, selectedVariableHolder, dataEntryHolder, timeDataHolder, jFrame);

      int nPoints = 200;
      YoVariableRegistry registry = new YoVariableRegistry("registry");
      YoDouble yoVariable = new YoDouble("variableOne", registry);
      
      DataBufferEntry dataEntry = new DataBufferEntry(yoVariable, nPoints);
      
      double value = 0.0;
      
      for (int i=0; i<nPoints; i++)
      {
         yoVariable.set(value);
         value = value + 0.001;
         dataEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      yoGraph.addVariable(dataEntry);

      jFrame.getContentPane().add(yoGraph);
      jFrame.setSize(800, 200);
      jFrame.setVisible(true);

//      while (true)
//      {
//         try
//         {
//            Thread.sleep(1000);
//         }
//         catch (InterruptedException e)
//         {
//         }
//
//      }
   }

   
   public static void main(String[] args)
   {
      new YoGraphTester().testYoGraph();
   }
}
