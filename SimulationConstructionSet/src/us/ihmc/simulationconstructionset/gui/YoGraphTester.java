package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import javax.swing.JFrame;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBufferEntry;

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
      DoubleYoVariable yoVariable = new DoubleYoVariable("variableOne", registry);
      
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

   
   private class MinimalTimeDataHolder implements TimeDataHolder
   {
      private double[] timeData;
      
      MinimalTimeDataHolder(int nPoints)
      {
         timeData = new double[nPoints];
         double time = 1.0;
         
         for (int i=0; i<nPoints; i++)
         {
            timeData[i] = time;
            time = time + 0.01;
         }
      }
      
      @Override
      public double[] getTimeData()
      {
         return timeData;
      }
      
   }
   
   private class MinimalGraphIndicesHolder implements GraphIndicesHolder
   {
      ArrayList<Integer> keyPoints = new ArrayList<Integer>();
      
      int leftPlotIndex = 0;
      int rightPlotIndex = 100;
      int inPoint = 50;
      int index = 60;
      int outPoint = 75;
      int maxIndex = 200;
      
      @Override
      public void tickLater(int i)
      {
         index = index + i;
      }

      @Override
      public void setRightPlotIndex(int newRightIndex)
      {
         this.rightPlotIndex = newRightIndex;
      }

      @Override
      public void setLeftPlotIndex(int newLeftIndex)
      {
         this.leftPlotIndex = newLeftIndex;
      }

      @Override
      public void setIndexLater(int newIndex)
      {
         this.index = newIndex;
      }

      @Override
      public int getRightPlotIndex()
      {
         return rightPlotIndex;
      }

      @Override
      public int getMaxIndex()
      {
         return maxIndex;
      }

      @Override
      public int getLeftPlotIndex()
      {
         return leftPlotIndex;
      }

      @Override
      public ArrayList<Integer> getKeyPoints()
      {
         return keyPoints;
      }

      @Override
      public int getIndex()
      {
         return index;
      }

      @Override
      public int getInPoint()
      {
         return inPoint;
      }
      
      @Override
      public int getOutPoint()
      {
         return outPoint;
      }

      @Override
      public boolean isIndexAtOutPoint()
      {
         return (getIndex() == getOutPoint());
      }
   }

   public static void main(String[] args)
   {
      new YoGraphTester().testYoGraph();
   }
}
