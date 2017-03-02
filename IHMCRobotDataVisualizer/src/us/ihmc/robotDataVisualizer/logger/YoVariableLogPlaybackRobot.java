package us.ihmc.robotDataVisualizer.logger;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.LogIndex;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataVisualizer.VisualizerRobot;
import us.ihmc.robotDataVisualizer.visualizer.JointUpdater;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableLogPlaybackRobot extends VisualizerRobot implements RewoundListener
{

   private final SimulationConstructionSet scs;
   private final LongYoVariable timestamp;
   private final DoubleYoVariable robotTime;
   private final FileChannel logChannel;
   private final List<YoVariable<?>> variables;

   // Compressed data helpers
   private final boolean compressed;
   private final LogIndex logIndex;
   private final ByteBuffer compressedBuffer;
   private int index = 0;

   private final List<JointState> jointStates;
   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<JointUpdater>();

   private final ArrayList<YoVariableLogPlaybackListener> listeners = new ArrayList<>();

   private final ByteBuffer logLine;
   private final LongBuffer logLongArray;

   private final IntegerYoVariable currentRecordTick;

   private final int numberOfEntries;
   private final long initialTimestamp;
   private final long finalTimestamp = 0;

   private int readEveryNTicks = 1;

   public YoVariableLogPlaybackRobot(File selectedFile, RobotDescription robotDescription,
         List<JointState> jointStates, List<YoVariable<?>> variables, LogPropertiesReader logProperties, SimulationConstructionSet scs)
         throws IOException
   {
      super(robotDescription);

      this.timestamp = new LongYoVariable("timestamp", getRobotsYoVariableRegistry());
      this.robotTime = new DoubleYoVariable("robotTime", getRobotsYoVariableRegistry());



      this.jointStates = jointStates;
      this.variables = variables;
      this.scs = scs;

      int jointStateOffset = variables.size();
      int numberOfJointStates = JointState.getNumberOfJointStates(jointStates);
      int bufferSize = (1 + jointStateOffset + numberOfJointStates) * 8;

      File logdata = new File(selectedFile, logProperties.getVariableDataFile());
      if (!logdata.exists())
      {
         throw new RuntimeException("Cannot find " + logProperties.getVariableDataFile());
      }
      this.logChannel = new FileInputStream(logdata).getChannel();

      this.compressed = logProperties.getCompressed();
      if (this.compressed)
      {
         File indexData = new File(selectedFile, logProperties.getVariablesIndexFile());
         if (!indexData.exists())
         {
            throw new RuntimeException("Cannot find " + logProperties.getVariablesIndexFile());
         }
         logIndex = new LogIndex(indexData, logChannel.size());
         compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
         numberOfEntries = logIndex.getNumberOfEntries();
      }
      else
      {
         numberOfEntries = (int) (logChannel.size() / bufferSize) - 1;
         logIndex = null;
         compressedBuffer = null;
      }

      JointUpdater.getJointUpdaterList(getRootJoints(), jointStates, jointUpdaters);

      logLine = ByteBuffer.allocate(bufferSize);
      logLongArray = logLine.asLongBuffer();

      currentRecordTick = new IntegerYoVariable("currentRecordTick", getRobotsYoVariableRegistry());

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

      scs.setRobot(this);
      scs.attachSimulationRewoundListener(this);
   }

   public long getInitialTimestamp()
   {
      return initialTimestamp;
   }

   public long getFinalTimestamp()
   {
      return finalTimestamp;
   }

   public int getNumberOfEntries()
   {
      return numberOfEntries;
   }

   public LongYoVariable getTimestamp()
   {
      return timestamp;
   }

   public void seek(int position)
   {
      currentRecordTick.set(position);
      try
      {
         positionChannel(position);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void setReadEveryNTicks(int readEveryNTicks)
   {
      if (readEveryNTicks < 1)
         readEveryNTicks = 1;
      this.readEveryNTicks = readEveryNTicks;
   }

   public int getReadEveryNTicks()
   {
      return readEveryNTicks;
   }

   // By overriding this method, we make the simulate button be the read data in button.
   @Override
   public void doDynamicsAndIntegrate(double DT)
   {
      for (int i = 0; i < readEveryNTicks; i++)
      {
         boolean done = readAndProcessALogLineReturnTrueIfDone(DT);
         if (done)
            return;
         currentRecordTick.increment();
      }

      update();
   }

   private boolean readAndProcessALogLineReturnTrueIfDone(double DT)
   {
      try
      {
         if (!readLogLine())
         {
            System.out.println("Reached end of file, stopping simulation thread");
            scs.stop();
            return true;
         }

         timestamp.set(logLongArray.get());
         robotTime.set(Conversions.nanoSecondstoSeconds(timestamp.getLongValue() - initialTimestamp));

         for (int i = 0; i < variables.size(); i++)
         {
            YoVariable<?> variable = variables.get(i);
            variable.setValueFromLongBits(logLongArray.get(), true);
         }

         for (int i = 0; i < jointStates.size(); i++)
         {
            jointStates.get(i).update(logLongArray);
         }

         for (int i = 0; i < jointUpdaters.size(); i++)
         {
            jointUpdaters.get(i).update();
         }

         for (int i = 0; i < listeners.size(); i++)
         {
            listeners.get(i).updated(timestamp.getLongValue());
         }

      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      t.add(DT);
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

   public long getTimestamp(int position)
   {
      if(!compressed)
      {
         throw new RuntimeException("Cannot get timestamp for non-compressed logs");
      }

      return logIndex.timestamps[position];
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

   public void addCurrentRecordTickListener(VariableChangedListener listener)
   {
      currentRecordTick.addVariableChangedListener(listener);
   }

   public void wasRewound()
   {
      int position = currentRecordTick.getIntegerValue();
      try
      {
         positionChannel(position);
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot skip to position " + position);
      }
   }



   public void addLogPlaybackListener(YoVariableLogPlaybackListener listener)
   {
      listener.setRobot(this);
      listeners.add(listener);
   }


}
