package us.ihmc.gdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.gdx.simulation.scs2.GDXYoManager;
import us.ihmc.gdx.ui.tools.ImPlotTools;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImPlotYoBufferIntegerPlotLine extends ImPlotYoBufferPlotLineBasics
{
   private final YoInteger yoInteger;
   private LinkedYoVariable<YoInteger> linkedYoIntegerVariable;
   private Integer[] xValues = ImPlotTools.createIndex(1);
   private Integer[] plotData = ImPlotTools.newZeroFilledIntegerBuffer(1);

   public ImPlotYoBufferIntegerPlotLine(YoInteger yoInteger, Consumer<YoVariable> removeSelf)
   {
      super(yoInteger, "0", removeSelf);
      this.yoInteger = yoInteger;
   }

   @Override
   public void setupLinkedVariable(GDXYoManager yoManager)
   {
      if (linkedYoIntegerVariable == null)
      {
         linkedYoIntegerVariable = (LinkedYoVariable<YoInteger>) yoManager.newLinkedYoVariable(yoInteger);
         linkedYoIntegerVariable.addUser(this);
      }
   }

   @Override
   public void update()
   {
      if (linkedYoIntegerVariable != null)
      {
         linkedYoIntegerVariable.pull();

         if (linkedYoIntegerVariable.isRequestedBufferSampleAvailable())
         {
            BufferSample<int[]> bufferSample = linkedYoIntegerVariable.pollRequestedBufferSample();
            int[] buffer = bufferSample.getSample();
            int sampleLength = bufferSample.getSampleLength();
            if (plotData.length != sampleLength)
            {
               xValues = ImPlotTools.createIndex(sampleLength);
               plotData = ImPlotTools.newZeroFilledIntegerBuffer(sampleLength);
            }
            for (int i = 0; i < bufferSample.getBufferProperties().getActiveBufferLength(); i++)
            {
               plotData[i] = buffer[i];
            }
         }

         linkedYoIntegerVariable.requestEntireBuffer();
      }
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
