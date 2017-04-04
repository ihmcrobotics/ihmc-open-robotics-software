package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;

public class MinimalTimeDataHolder implements TimeDataHolder
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