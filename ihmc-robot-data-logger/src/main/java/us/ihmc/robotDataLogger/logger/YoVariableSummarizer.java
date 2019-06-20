package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableSummarizer
{
   private final int trigger;
   private final YoVariable<?> triggerVariable;
   private final YoVariableSummarizerData[] variables;

   private LongBuffer buffer;

   public YoVariableSummarizer(List<YoVariable<?>> yoVariables, String triggerVariable, String[] variables)
   {
      trigger = getYoVariable(yoVariables, triggerVariable);
      if (trigger == -1)
      {
         LogTools.error("No trigger variable found. Summarizing all data points.");
      }
      this.triggerVariable = yoVariables.get(trigger);
      LogTools.info("Creating summary of variables.\nTrigger Variable = " + this.triggerVariable.getName());
      

      ArrayList<YoVariableSummarizerData> summaryVariables = new ArrayList<>();
      for (String variable : variables)
      {
         int summaryVariable = getYoVariable(yoVariables, variable);
         if (summaryVariable == -1)
         {
            LogTools.error("Cannot find variable " + variable + " for summarizing.");
         }
         else
         {
            summaryVariables.add(new YoVariableSummarizerData(summaryVariable, yoVariables.get(summaryVariable)));
         }
      }

      this.variables = summaryVariables.toArray(new YoVariableSummarizerData[summaryVariables.size()]);
   }

   private void updateVariable(int offset, YoVariable<?> variable)
   {
      variable.setValueFromLongBits(buffer.get(offset), false);
   }

   public int getYoVariable(List<YoVariable<?>> yoVariables, String name)
   {
      for (int i = 0; i < yoVariables.size(); i++)
      {
         if (yoVariables.get(i).getFullNameWithNameSpace().endsWith(name))
         {
            return i;
         }
      }
      return -1;
   }

   public void update()
   {
      if (triggerVariable != null)
      {
         updateVariable(trigger, triggerVariable);
         if (triggerVariable.isZero())
         {
            return;
         }
      }
      for (YoVariableSummarizerData data : variables)
      {
         data.update();
      }
   }

   public void writeData(File file)
   {
      try
      {
         PrintWriter writer = new PrintWriter(file);
         writer.print("namespace");
         writer.print(',');
         writer.print("name");
         writer.print(',');
         writer.print("minimum");
         writer.print(',');
         writer.print("maximum");
         writer.print(',');
         writer.print("average");
         writer.print(',');
         writer.print("delta");
         writer.println();
         for (YoVariableSummarizerData data : variables)
         {
            data.writeCSV(writer);
         }
         writer.flush();
         writer.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
   }

   private class YoVariableSummarizerData
   {
      private final int variableOffset;
      private final YoVariable<?> variable;
      private double minimum;
      private double maximum;

      private double average;
      private long samples;

      public YoVariableSummarizerData(int variableOffset, YoVariable<?> variable)
      {
         this.variableOffset = variableOffset;
         this.variable = variable;
         clear();
      }

      public void update()
      {
         updateVariable(variableOffset, variable);
         if (variable.getValueAsDouble() < minimum)
         {
            minimum = variable.getValueAsDouble();
         }
         if (variable.getValueAsDouble() > maximum)
         {
            maximum = variable.getValueAsDouble();
         }

         samples++;
         average += (variable.getValueAsDouble() - average) / (samples);
      }

      public void writeCSV(PrintWriter writer)
      {
         writer.print(variable.getNameSpace());
         writer.print(',');
         writer.print(variable.getName());
         writer.print(',');
         writer.print(minimum);
         writer.print(',');
         writer.print(maximum);
         writer.print(',');
         writer.print(average);
         writer.print(',');
         writer.print(maximum - minimum);
         writer.println();

      }

      public void clear()
      {
         minimum = Double.POSITIVE_INFINITY;
         maximum = Double.NEGATIVE_INFINITY;

         average = 0.0;
         samples = 0;

      }
   }

   public void restart()
   {
      for (YoVariableSummarizerData data : variables)
      {
         data.clear();
      }
   }

   public void setBuffer(ByteBuffer buffer)
   {
      this.buffer = buffer.asLongBuffer();
   }
}
