package us.ihmc.rdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.rdx.imgui.ImPlotTools;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImPlotYoBufferEnumPlotLine extends ImPlotYoBufferPlotLineBasics
{
   private final YoEnum yoEnum;
   private LinkedYoVariable<YoEnum> linkedYoEnumVariable;
   private double[] xValues = ImPlotTools.createIndex(1);
   private double[] plotData = ImPlotTools.newZeroFilledBuffer(1);

   public ImPlotYoBufferEnumPlotLine(YoEnum yoEnum, Consumer<YoVariable> removeSelf)
   {
      super(yoEnum, "0", removeSelf);
      this.yoEnum = yoEnum;
   }

   @Override
   public void setupLinkedVariable(RDXYoManager yoManager)
   {
      if (linkedYoEnumVariable == null)
      {
         linkedYoEnumVariable = (LinkedYoVariable<YoEnum>) yoManager.newLinkedYoVariable(yoEnum);
         linkedYoEnumVariable.addUser(this);
      }
   }

   @Override
   public void update()
   {
      if (linkedYoEnumVariable != null)
      {
         linkedYoEnumVariable.pull();

         if (linkedYoEnumVariable.isRequestedBufferSampleAvailable())
         {
            BufferSample<byte[]> bufferSample = linkedYoEnumVariable.pollRequestedBufferSample();
            byte[] buffer = bufferSample.getSample();
            int sampleLength = bufferSample.getSampleLength();
            if (plotData.length != sampleLength)
            {
               xValues = ImPlotTools.createIndex(sampleLength);
               plotData = ImPlotTools.newZeroFilledBuffer(sampleLength);
            }
            for (int i = 0; i < bufferSample.getBufferProperties().getActiveBufferLength(); i++)
            {
               plotData[i] = buffer[i];
            }
         }

         linkedYoEnumVariable.requestEntireBuffer();
      }
   }

   @Override
   protected void plot(String labelID)
   {
      int offset = 0; // This is believed to be the index in the array we are passing in which implot will start reading
      ImPlot.plotLine(labelID, xValues, plotData, xValues.length, offset);
   }

   @Override
   public String getValueString(int bufferIndex)
   {
      return plotData[bufferIndex] > -1 ? yoEnum.getEnumValuesAsString()[(int) plotData[bufferIndex]] : "null";
   }
}
