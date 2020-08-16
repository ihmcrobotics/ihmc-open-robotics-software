package us.ihmc.robotDataVisualizer.logger;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.jmatio.io.MatFileWriter;
import com.jmatio.types.MLArray;
import com.jmatio.types.MLDouble;
import com.jmatio.types.MLInt64;
import com.jmatio.types.MLNumericArray;

import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;
import us.ihmc.robotDataVisualizer.logger.util.CustomProgressMonitor;
import us.ihmc.robotDataVisualizer.logger.util.ProgressMonitorInterface;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.yoVariables.variable.*;

public class YoVariableExporter extends YoVariableLogReader
{
   private final List<YoVariable> variables;
   private final Map<String, YoVariable> fullnameToVariableMap;
   private final Map<String, List<YoVariable>> nameToVariablesMap = new HashMap<>();

   public YoVariableExporter(SimulationConstructionSet scs, File logDirectory, LogProperties logProperties, List<YoVariable> variables)
   {
      super(logDirectory, logProperties);
      this.variables = variables;
      fullnameToVariableMap = variables.stream().collect(Collectors.toMap(YoVariable::getFullNameString, Function.identity()));
      for (YoVariable variable : variables)
      {
         String name = variable.getName();
         List<YoVariable> variableList = nameToVariablesMap.get(name);

         if (variableList == null)
         {
            variableList = new ArrayList<>();
            nameToVariablesMap.put(name, variableList);
         }

         variableList.add(variable);
      }
   }

   public void exportMatlabData(File file, long start, long end, VarGroup vargroup)
   {
      ProgressMonitorInterface monitor = new CustomProgressMonitor("Export data to Matlab", "Reading variable data", 0, 100);

      if (!initialize())
      {
         return;
      }

      try
      {
         int startPosition = getPosition(start);
         int endPosition = getPosition(end);
         int elements = endPosition - startPosition + 1;

         // Time element
         MLInt64 timestamp = new MLInt64("timestamp", new int[] {elements, 1});
         MLDouble robotTime = new MLDouble("robotTime", new int[] {elements, 1});

         List<DataHolder<?>> dataHolders = toDataHolders(elements, vargroup);

         int step = elements / 90;

         long firstTimestamp = -1;
         for (int i = startPosition; i <= endPosition; i++)
         {
            if ((i - startPosition) % step == 0)
            {
               monitor.setProgress((i - startPosition) / step);
            }

            ByteBuffer data = readData(i);
            LongBuffer dataAsLong = data.asLongBuffer();

            long entryTimestamp = dataAsLong.get();

            if (firstTimestamp == -1)
            {
               firstTimestamp = entryTimestamp;
            }

            timestamp.setReal(entryTimestamp, i - startPosition);
            robotTime.setReal(Conversions.nanosecondsToSeconds(entryTimestamp - firstTimestamp), i - startPosition);
            dataHolders.forEach(dataHolder -> dataHolder.addEntry(dataAsLong));
         }

         monitor.setNote("Writing data to disk");
         ArrayList<MLArray> matlabData = new ArrayList<>();

         matlabData.add(timestamp);
         matlabData.add(robotTime);
         dataHolders.forEach(dataHolder -> matlabData.add(dataHolder.getData()));

         new MatFileWriter(file, matlabData);

         monitor.close();

      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private List<DataHolder<?>> toDataHolders(int elements, VarGroup vargroup)
   {
      List<DataHolder<?>> dataHolders = new ArrayList<>();

      if (vargroup == null)
      {
         for (int i = 0; i < variables.size(); i++)
         {
            dataHolders.add(createDataHolder(i + 1, elements, variables.get(i)));
         }
      }
      else
      {
         for (String varname : vargroup.getVars())
         {
            List<YoVariable> variableList = findVariable(varname);

            if (variableList == null || variableList.isEmpty())
            {
               LogTools.warn("Could not find variable for " + varname);
               continue;
            }

            if (variableList.size() > 1)
            {
               LogTools.info("Found multiple variables for " + varname);
            }

            for (YoVariable variable : variableList)
            {
               int offset = variables.indexOf(variable);
               if (offset == -1)
                  throw new IllegalStateException("Should not get here");
               offset++;
               dataHolders.add(createDataHolder(offset, elements, variable));
            }
         }
      }

      if (dataHolders.isEmpty())
         return null;
      else
         return dataHolders;
   }

   private List<YoVariable> findVariable(String varName)
   {
      YoVariable variable = fullnameToVariableMap.get(varName);

      if (variable != null)
         return Collections.singletonList(variable);
      else
         return nameToVariablesMap.get(varName);
   }

   private DataHolder<?> createDataHolder(int offset, int elements, YoVariable variable)
   {
      int[] dims = {elements, 1};
      String name = variable.getName();
      if (variable instanceof YoEnum<?>)
      {
         return new DataHolder<Long>(offset, new MLInt64(name, dims))
         {
            @Override
            public void set(long entryAsLong)
            {
               set((Long) entryAsLong);
            }

         };
      }
      else if (variable instanceof YoLong)
      {
         return new DataHolder<Long>(offset, new MLInt64(name, dims))
         {
            @Override
            public void set(long entryAsLong)
            {
               set((Long) entryAsLong);
            }

         };
      }
      else if (variable instanceof YoInteger)
      {
         return new DataHolder<Long>(offset, new MLInt64(name, dims))
         {
            @Override
            public void set(long entryAsLong)
            {
               set((Long) entryAsLong);
            }

         };
      }
      else if (variable instanceof YoDouble)
      {
         return new DataHolder<Double>(offset, new MLDouble(name, dims))
         {
            @Override
            public void set(long entryAsLong)
            {
               set(Double.longBitsToDouble(entryAsLong));
            }

         };
      }
      else if (variable instanceof YoBoolean)
      {
         return new DataHolder<Long>(offset, new MLInt64(name, dims))
         {
            @Override
            public void set(long entryAsLong)
            {
               set((Long) (entryAsLong == 0L ? 0L : 1L)); // Force true to equal 1L in all cases
            }
         };
      }
      else
      {
         throw new RuntimeException("Unknown YoVariable type " + variable.getClass().getSimpleName());
      }
   }

   private abstract class DataHolder<T extends Number>
   {
      private final MLNumericArray<T> data;
      private final int offset;

      private int index = 0;

      private DataHolder(int offset, MLNumericArray<T> data)
      {
         this.data = data;
         this.offset = offset;
      }

      public void addEntry(LongBuffer dataAsLong)
      {
         long entryAsLong = dataAsLong.get(offset);
         set(entryAsLong);
         index++;
      }

      public abstract void set(long entryAsLong);

      protected void set(T entryAsDataType)
      {
         data.setReal(entryAsDataType, index);
      }

      public MLNumericArray<T> getData()
      {
         return data;
      }
   }

}
