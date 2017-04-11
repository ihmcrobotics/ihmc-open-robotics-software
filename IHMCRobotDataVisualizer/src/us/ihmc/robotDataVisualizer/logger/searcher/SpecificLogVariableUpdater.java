package us.ihmc.robotDataVisualizer.logger.searcher;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.LogIndex;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.tools.compression.SnappyUtils;

public class SpecificLogVariableUpdater
{
   private final YoVariableRegistry registry = new YoVariableRegistry("YoVariableSpecificLogVariablePlaybackRobot");
   private final LongYoVariable timestamp = new LongYoVariable("timestamp", registry);
   private final DoubleYoVariable robotTime = new DoubleYoVariable("robotTime", registry);

   private final FileChannel logChannel;
   private final List<YoVariable<?>> variables;

   // Compressed data helpers
   private final boolean compressed;
   private final LogIndex logIndex;
   private final ByteBuffer compressedBuffer;
   private int index = 0;

   private final ByteBuffer logLine;
   private final LongBuffer logLongArray;

   private final long initialTimestamp;

   private YoVariable<?>[] variablesToUpdate;
   private final HashMap<YoVariable<?>, AtomicInteger> indexes = new HashMap<>();

   public SpecificLogVariableUpdater(File selectedFile, RobotDescription robotDescription,
         List<JointState> jointStates, List<YoVariable<?>> variables, LogPropertiesReader logProperties, YoVariable<?>... variablesToUpdate )
         throws IOException
   {
      this.variables = variables;
      this.variablesToUpdate = variablesToUpdate;

      for(YoVariable<?> yoVariable : variablesToUpdate)
      {
         indexes.put(yoVariable, new AtomicInteger(0));
      }

      int jointStateOffset = variables.size();
      int numberOfJointStates = JointState.getNumberOfJointStates(jointStates);
      int bufferSize = (1 + jointStateOffset + numberOfJointStates) * 8;

      File logdata = new File(selectedFile, logProperties.getVariables().getDataAsString());
      if (!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariables().getDataAsString());
      }
      this.logChannel = new FileInputStream(logdata).getChannel();

      this.compressed = logProperties.getVariables().getCompressed();
      if (this.compressed)
      {
         File indexData = new File(selectedFile, logProperties.getVariables().getIndexAsString());
         if (!indexData.exists())
         {
            throw new RuntimeException("Cannot find " + logProperties.getVariables().getIndexAsString());
         }
         logIndex = new LogIndex(indexData, logChannel.size());
         compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
      }
      else
      {
         logIndex = null;
         compressedBuffer = null;
      }

      logLine = ByteBuffer.allocate(bufferSize);
      logLongArray = logLine.asLongBuffer();

      try
      {
      	if(this.compressed)
      	{
         	initialTimestamp = logIndex.getInitialTimestamp();
         	positionChannel(0);
         }
         else
         {
         	readLogLine();
         	initialTimestamp = logLine.getLong(0);
         	positionChannel(0);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      getIndexes();
   }

   public void getIndexes()
   {
      timestamp.set(logLongArray.get());
      int startPositoin = logLongArray.position();
         
      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable<?> variable = variables.get(i);
         if(indexes.containsKey(variable))
         {
            indexes.get(variable).set(logLongArray.position());
         }
         variable.setValueFromLongBits(logLongArray.get(), true);
      }
      logLongArray.position(startPositoin);
   }

   public boolean readAndProcessALogLineReturnTrueIfDone(double DT)
   {
      try
      {
         if (!readLogLine())
         {
            return true;
         }

         timestamp.set(logLongArray.get());
         robotTime.set(Conversions.nanosecondsToSeconds(timestamp.getLongValue() - initialTimestamp));

         for (int i = 0; i < variablesToUpdate.length; i++)
         {
            YoVariable<?> variable = variablesToUpdate[i];
            AtomicInteger position = indexes.get(variable);
            
            variable.setValueFromLongBits(logLongArray.get(position.get()), false);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return false;
   }

   private void positionChannel(int position) throws IOException
   {
      if (compressed)
      {
         index = position;
         if(index < logIndex.dataOffsets.length)
         {
            logChannel.position(logIndex.dataOffsets[position]);
         }
      }
      else
      {
         logChannel.position(((long)position) * ((long)logLine.capacity()));
      }
   }

   private boolean readLogLine() throws IOException
   {
      logLine.clear();
      logLongArray.clear();

      if (compressed)
      {
         if(index >= logIndex.getNumberOfEntries())
         {
            return false;
         }
         int size = logIndex.compressedSizes[index];
         compressedBuffer.clear();
         compressedBuffer.limit(size);

         int read = logChannel.read(compressedBuffer);

         if(read != size)
         {
            throw new RuntimeException("Expected read of " + size + ", got " + read + ". TODO: Implement loop for reading the full log line.");
         }
         compressedBuffer.flip();

         SnappyUtils.uncompress(compressedBuffer, logLine);
         ++index;

         return true;
      }
      else
      {
         int read = logChannel.read(logLine);
         if(read < 0)
         {
            return false;
         }
         else if (read != logLine.capacity())
         {
            throw new RuntimeException("Expected read of " + logLine.capacity() + ", got " + read + ". TODO: Implement loop for reading the full log line.");
         }
         else
         {
            return true;
         }

      }
   }

   public void close()
   {
      try
      {
         logChannel.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public double getTime()
   {
      return robotTime.getDoubleValue();
   }
}
