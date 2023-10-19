package us.ihmc.rdx.ui.yo;

import imgui.extension.implot.ImPlot;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.rdx.imgui.ImPlotTools;
import us.ihmc.scs2.sharedMemory.BufferSample;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.function.Consumer;

public class ImPlotYoBufferBooleanPlotLine extends ImPlotYoBufferPlotLineBasics
{
   private final YoBoolean yoBoolean;
   private LinkedYoVariable<YoBoolean> linkedYoBooleanVariable;
   private double[] xValues = ImPlotTools.createIndex(1);
   private double[] plotData = ImPlotTools.newZeroFilledBuffer(1);

   public ImPlotYoBufferBooleanPlotLine(YoBoolean yoBoolean, Consumer<YoVariable> removeSelf)
   {
      super(yoBoolean, "0", removeSelf);
      this.yoBoolean = yoBoolean;
   }

   @Override
   public void setupLinkedVariable(RDXYoManager yoManager)
   {
      if (linkedYoBooleanVariable == null)
      {
         linkedYoBooleanVariable = (LinkedYoVariable<YoBoolean>) yoManager.newLinkedYoVariable(yoBoolean);
         linkedYoBooleanVariable.addUser(this);
      }
   }

   @Override
   public void update()
   {
      if (linkedYoBooleanVariable != null)
      {
         linkedYoBooleanVariable.pull();

         if (linkedYoBooleanVariable.isRequestedBufferSampleAvailable())
         {
            BufferSample<boolean[]> bufferSample = linkedYoBooleanVariable.pollRequestedBufferSample();
            boolean[] buffer = bufferSample.getSample();
            int sampleLength = bufferSample.getSampleLength();
            if (plotData.length != sampleLength)
            {
               xValues = ImPlotTools.createIndex(sampleLength);
               plotData = ImPlotTools.newZeroFilledBuffer(sampleLength);
            }
            for (int i = 0; i < bufferSample.getBufferProperties().getActiveBufferLength(); i++)
            {
               plotData[i] = buffer[i] ? 1 : 0;
            }
         }

         linkedYoBooleanVariable.requestEntireBuffer();
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
      return String.valueOf(plotData[bufferIndex]);
   }
}
