package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.gdx.ui.tools.ImPlotTools;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoInteger;

public class ImPlotYoBufferIntegerPlotLine extends ImPlotYoBufferPlotLineBasics
{
   private final LinkedYoVariable<YoInteger> linkedYoIntegerVariable;
   private Integer[] xValues = ImPlotTools.createIndex(1);
   private Integer[] plotData = ImPlotTools.newZeroFilledIntegerBuffer(1);

   public ImPlotYoBufferIntegerPlotLine(LinkedYoVariable<YoInteger> linkedYoIntegerVariable)
   {
      super(linkedYoIntegerVariable.getLinkedYoVariable().getName(), "0");
      this.linkedYoIntegerVariable = linkedYoIntegerVariable;
      linkedYoIntegerVariable.addUser(this);
   }

   @Override
   public void update()
   {
      linkedYoIntegerVariable.pull();

      if (linkedYoIntegerVariable.isRequestedBufferSampleAvailable())
      {
         BufferSample<int[]> bufferSample = linkedYoIntegerVariable.pollRequestedBufferSample();
         int[] buffer = bufferSample.getSample();
         int activeBufferLength = bufferSample.getBufferProperties().getActiveBufferLength();
         if (plotData.length != activeBufferLength)
         {
            xValues = ImPlotTools.createIndex(activeBufferLength);
            plotData = ImPlotTools.newZeroFilledIntegerBuffer(activeBufferLength);
         }
         for (int i = 0; i < bufferSample.getBufferProperties().getSize(); i++)
         {
            plotData[i] = buffer[i];
         }
      }

      linkedYoIntegerVariable.requestEntireBuffer();
   }

   @Override
   protected void plot(String labelID)
   {
      int offset = 0; // This is believed to be the index in the array we are passing in which implot will start reading
      ImPlot.plotLine(labelID, xValues, plotData, offset);
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return String.valueOf(plotData[bufferIndex]);
   }
}
