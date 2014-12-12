package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.robotDataCommunication.visualizer.JointUpdater;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.compression.SnappyUtils;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.yoUtilities.dataStructure.listener.RewoundListener;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class YoVariableLogPlaybackRobot extends SDFRobot implements RewoundListener
{
   private final YoVariableRegistry logRegistry;
   
   private final SimulationConstructionSet scs;
   private final LongYoVariable timestamp;
   private final DoubleYoVariable robotTime;
   private final FileChannel logChannel;
   private final List<YoVariable<?>> variables;

   // Compressed data helpers
   private final boolean compressed;
   private final long[] dataOffsets;
   private final int[] compressedSizes;
   private final ByteBuffer compressedBuffer;
   private int index = 0;

   private final List<JointState<? extends Joint>> jointStates;
   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<JointUpdater>();

   private final ByteBuffer logLine;
   private final LongBuffer logLongArray;

   private final IntegerYoVariable currentRecordTick;

   private final int numberOfEntries;
   private final long initialTimestamp;
   private final long finalTimestamp = 0;

   private int readEveryNTicks = 1;

   public YoVariableLogPlaybackRobot(File selectedFile, GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap,
         List<JointState<? extends Joint>> jointStates, List<YoVariable<?>> variables, LogPropertiesReader logProperties, SimulationConstructionSet scs)
         throws IOException
   {
      super(generalizedSDFRobotModel, sdfJointNameMap, false);
      this.logRegistry = new YoVariableRegistry(generalizedSDFRobotModel.getName());
      
      this.timestamp = new LongYoVariable("timestamp", getRobotsYoVariableRegistry());
      this.robotTime = new DoubleYoVariable("robotTime", getRobotsYoVariableRegistry());
      
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQx());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQy());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQz());
      
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQdx());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQdy());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQdz());


      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQuaternionQs());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQuaternionQx());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQuaternionQy());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getQuaternionQz());
      
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getAngularVelocityX());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getAngularVelocityY());
      getRobotsYoVariableRegistry().registerVariable(getRootJoint().getAngularVelocityZ());
      
      for(OneDegreeOfFreedomJoint joint : getOneDoFJoints())
      {
         getRobotsYoVariableRegistry().registerVariable(joint.getQ());
         getRobotsYoVariableRegistry().registerVariable(joint.getQD());
      }
      
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
         @SuppressWarnings("resource")
         FileChannel indexChannel = new FileInputStream(indexData).getChannel();
         this.dataOffsets = new long[(int) (indexData.length() / 8)];
         ByteBuffer dataOffsetWrap = ByteBuffer.allocateDirect(dataOffsets.length * 8);
         indexChannel.read(dataOffsetWrap);
         dataOffsetWrap.flip();
         dataOffsetWrap.asLongBuffer().get(dataOffsets);
         indexChannel.close();

         compressedSizes = new int[dataOffsets.length];
         for (int i = 0; i < dataOffsets.length - 1; i++)
         {
            compressedSizes[i] = (int) (dataOffsets[i + 1] - dataOffsets[i]);
         }
         compressedSizes[dataOffsets.length - 1] = (int) (logChannel.size() - dataOffsets[dataOffsets.length - 1]);
         compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
         numberOfEntries = dataOffsets.length;
      }
      else
      {
         numberOfEntries = (int) (logChannel.size() / bufferSize) - 1;
         dataOffsets = null;
         compressedSizes = null;
         compressedBuffer = null;
      }

      JointUpdater.getJointUpdaterList(getRootJoints(), jointStates, jointUpdaters);

      logLine = ByteBuffer.allocate(bufferSize);
      logLongArray = logLine.asLongBuffer();

      currentRecordTick = new IntegerYoVariable("currentRecordTick", getRobotsYoVariableRegistry());

      try
      {
         readLogLine();
         initialTimestamp = logLine.getLong(0);
         positionChannel(0);
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
         robotTime.set(TimeTools.nanoSecondstoSeconds(timestamp.getLongValue() - initialTimestamp));

         for (int i = 0; i < variables.size(); i++)
         {
            YoVariable<?> variable = variables.get(i);
            variable.setValueFromLongBits(logLongArray.get());
         }

         for (int i = 0; i < jointStates.size(); i++)
         {
            jointStates.get(i).update(logLongArray);
         }

         for (int i = 0; i < jointUpdaters.size(); i++)
         {
            jointUpdaters.get(i).update();
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
         if(index < dataOffsets.length)
         {
            logChannel.position(dataOffsets[position]);            
         }
      }
      else
      {
         logChannel.position(position * logLine.capacity());
      }
   }

   private boolean readLogLine() throws IOException
   {
      logLine.clear();
      logLongArray.clear();

      if (compressed)
      {
         if(index >= compressedSizes.length)
         {
            return false;
         }
         int size = compressedSizes[index];
         compressedBuffer.clear();
         compressedBuffer.limit(size);

         int read = logChannel.read(compressedBuffer);if(read != size)
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

   @Override
   public YoVariableRegistry getRobotsYoVariableRegistry()
   {
      if(this.logRegistry == null)
      {
         // Hack to avoid null registry errors on startup.
         return super.getRobotsYoVariableRegistry();
      }
      else
      {
         return this.logRegistry;
      }
   }
}
