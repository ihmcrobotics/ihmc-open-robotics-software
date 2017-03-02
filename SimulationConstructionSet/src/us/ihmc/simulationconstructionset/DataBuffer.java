package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.LinkedHashSet;

import us.ihmc.graphicsDescription.dataBuffer.DataEntryHolder;
import us.ihmc.graphicsDescription.dataBuffer.TimeDataHolder;
import us.ihmc.jMonkeyEngineToolkit.camera.TrackingDollyCameraController;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.commands.DataBufferCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandListener;
import us.ihmc.simulationconstructionset.gui.KeyPoints;
import us.ihmc.simulationconstructionset.gui.RegularExpression;
import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;

public class DataBuffer extends YoVariableHolderImplementation
        implements java.io.Serializable, DataBufferCommandsExecutor, ToggleKeyPointModeCommandExecutor, TimeDataHolder, DataEntryHolder
{
   private String timeVariableName = "t";

   public static final int MAX_LENGTH_SHORT_NAME = 20;

   private static final long serialVersionUID = 6736812894819363756L;
   private int inPoint = 0;

   private int index = 0;
   private int maxBufferSize = 16384;
   private int outPoint = 0;
   private ArrayList<RewoundListener> simulationRewoundListeners = null;
   private DoubleYoVariable t = null;
   private final LinkedHashSet<YoVariable<?>> yoVariableSet = new LinkedHashSet<YoVariable<?>>();
   private boolean wrapBuffer = false;    // Default to Expand, not Wrap!  true;

   public KeyPoints keyPoints = new KeyPoints();
   private ArrayList<DataBufferListener> dataBufferListeners = new ArrayList<DataBufferListener>();
   private int bufferSize;
   private ArrayList<DataBufferEntry> entries;
   private ArrayList<IndexChangedListener> indexChangedListeners;

   public ArrayList<ToggleKeyPointModeCommandListener> toggleKeyPointModeCommandListeners = new ArrayList<ToggleKeyPointModeCommandListener>();

   private boolean clearing = false;

   private boolean safeToManualyChangeIndex = true;

   public DataBuffer()
   {
      entries = new ArrayList<DataBufferEntry>();
   }

   @Override
   public void closeAndDispose()
   {
      dataBufferListeners.clear();
      dataBufferListeners = null;

      entries.clear();
      entries = null;

      index = -1;
   }

   public DataBuffer(int bufferSize)
   {
      entries = new ArrayList<DataBufferEntry>();
      this.bufferSize = bufferSize;
   }

   public void setSafeToChangeIndex(boolean safe)
   {
      this.safeToManualyChangeIndex = safe;
   }

   public boolean isSafeToChangeIndex()
   {
      return safeToManualyChangeIndex;
   }

   public int getBufferSize()
   {
      return this.bufferSize;
   }

   public int getMaxBufferSize()
   {
      return this.maxBufferSize;
   }

   public boolean getWrapBuffer()
   {
      return this.wrapBuffer;
   }

   public void addEntry(DataBufferEntry entry)
   {
      if (entry.getDataLength() != this.bufferSize)
         throw new RuntimeException("entry.getDataLength() != this.bufferSize");

      entries.add(entry);
   }

   public DataBufferEntry addVariable(YoVariable<?> newVariable, int nPoints) throws RepeatDataBufferEntryException
   {
      addVariableToHolder(newVariable);
      yoVariableSet.add(newVariable);

      DataBufferEntry entry = new DataBufferEntry(newVariable, nPoints);
      this.addEntry(entry);

      if (newVariable.getName().equals("t"))
      {
         t = (DoubleYoVariable) newVariable;
      }

      return entry;
   }

   public void addVariable(YoVariable<?> newVariable) throws RepeatDataBufferEntryException
   {
      addVariable(newVariable, bufferSize);
   }

   public void addVariables(ArrayList<YoVariable<?>> variables) throws RepeatDataBufferEntryException
   {
      entries.ensureCapacity(entries.size() + variables.size());    // do this first so that 'entries' will only have to grow once.

      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable<?> v = variables.get(i);

//       System.out.println("Adding YoVariable: " + v);

         this.addVariable(v);
      }
   }

   public void addDataBufferListener(DataBufferListener dataBufferListener)
   {
      this.dataBufferListeners.add(dataBufferListener);
   }

   public ArrayList<YoVariable<?>> getVariablesThatContain(String searchString, boolean caseSensitive, ArrayList<YoVariable<?>> currentlyMatched)
   {
      ArrayList<YoVariable<?>> ret = null;

      if (currentlyMatched != null)
      {
         if (!caseSensitive)
         {
            searchString = searchString.toLowerCase();
         }

         for (int i = 0; i < currentlyMatched.size(); i++)
         {
            YoVariable<?> entry = currentlyMatched.get(i);

            if (entry.getName().toLowerCase().contains((searchString)))
            {
               if (ret == null)
               {
                  ret = new ArrayList<YoVariable<?>>();
               }

               ret.add(entry);
            }
         }
      }

      return ret;
   }

   public ArrayList<YoVariable<?>> getVariablesThatStartWith(String searchString)
   {
      ArrayList<YoVariable<?>> ret = null;

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         if (entry.getVariable().getName().startsWith(searchString))
         {
            if (ret == null)
            {
               ret = new ArrayList<YoVariable<?>>();
            }

            ret.add(entry.getVariable());
         }
      }

      return ret;
   }

   public DataBufferEntry getEntry(String name)
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         YoVariable<?> variable = entry.getVariable();

         if (variable.fullNameEndsWithCaseInsensitive(name))
         {
            return entry;
         }
      }

      return null;
   }

   @Override
   public DataBufferEntry getEntry(YoVariable<?> v)
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         if (entry.getVariable() == v)
         {
            return entry;
         }
      }

      return null;
   }

   public ArrayList<DataBufferEntry> getEntries()
   {
      return this.entries;
   }

   public ArrayList<YoVariable<?>> getVariables()
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>(entries.size());

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         ret.add(entry.getVariable());
      }

      return ret;
   }

   public ArrayList<YoVariable<?>> getVars(String[] varNames, String[] regularExpressions)
   {
      YoVariableList tempList = new YoVariableList("temp");

      for (int i = 0; i < entries.size(); i++)
      {
         YoVariable<?> var = (entries.get(i)).getVariable();

         tempList.addVariable(var);
      }

      return tempList.getMatchingVariables(varNames, regularExpressions);
   }

   public ArrayList<YoVariable<?>> getVarsFromGroup(String varGroupName, VarGroupList varGroupList)
   {
      if (varGroupName.equals("all"))
      {
         return getVariables();
      }

      VarGroup varGroup = varGroupList.getVarGroup(varGroupName);
      String[] varNames = varGroup.getVars();
      String[] regularExpressions = varGroup.getRegularExpressions();

      return getVars(varNames, regularExpressions);
   }

   /**
    * Sets the maximum size, in ticks, to which the buffer will expand.  While nonsense values are not explicitly checked for, they will not cause the buffer to shrink.
    *
    * @param newMaxBufferSize New max buffer size.
    */
   public void setMaxBufferSize(int newMaxBufferSize)
   {
      this.maxBufferSize = newMaxBufferSize;
   }

   /**
    * Enables or disables buffer wrapping in place of buffer expansion.  By default the buffer will expand until it reaches maxBufferSize at which point it will wrap to the beginning.  When wrapBuffer is enabled the buffer wraps to the beginning without attempting to expand.
    *
    * @param newWrapBuffer Enable or disable wrap buffer mode.
    */
   public void setWrapBuffer(boolean newWrapBuffer)
   {
      this.wrapBuffer = newWrapBuffer;
   }

   public void resetDataBuffer()
   {
      clearAll(getBufferSize());

      if (!clearing)
      {
         setInPoint(0);
         gotoInPoint();
         clearAll(getBufferSize());
         tickAndUpdate();
      }
      else
      {
         setInPoint(0);
         clearing = true;
      }
   }

   public void clearAll(int nPoints)
   {
      double[] blankData;

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         blankData = new double[nPoints];
         entry.setData(blankData, nPoints);
      }

      this.bufferSize = nPoints;
   }

   public void changeBufferSize(int newBufferSize)
   {
      // if ((newBufferSize < 1) || (newBufferSize > maxBufferSize)) return;
      if (newBufferSize < bufferSize)
      {
         cropData(this.inPoint, ((this.inPoint + newBufferSize - 1) % bufferSize));
      }
      else if (newBufferSize > bufferSize)
      {
         packData();
         enlargeBufferSize(newBufferSize);
      }

      // bufferSize = newBufferSize;
   }

   private void enlargeBufferSize(int newSize)
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         entry.enlargeBufferSize(newSize);
      }

      bufferSize = newSize;
   }

   public void copyValuesThrough()
   {
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         entry.copyValueThrough();
      }
   }

   public int getBufferInOutLength()
   {
      int ret;

      if (outPoint > inPoint)
      {
         ret = outPoint - inPoint + 1;
      }
      else
      {
         ret = bufferSize - (inPoint - outPoint) + 1;
      }

      return ret;
   }

   public void packData()
   {
      packData(this.inPoint);
   }

   public void packData(int start)
   {
      if (start == 0)
         return;

      // If the start point is outside of the buffer abort.
      if ((start <= 0) || (start >= bufferSize))
      {
         return;
      }

      // Shift the data in each entry to begin with start.
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         entry.packData(start);
      }

      // Move the current index to its relative position in the new data set, if the index is outside of the buffer move to zero
      this.index = ((this.index - start + bufferSize) % bufferSize);

      if (this.index < 0)
      {
         this.index = 0;
      }

      // Move the inPoint to the new beginning and the outPoint to the end
      this.inPoint = 0;    // this.inPoint - start;
      this.outPoint = ((this.outPoint - start + bufferSize) % bufferSize);

      // Move to the first tick
      this.tick(0);

      // +++++this.updateUI();
   }

   public void cropData()
   {
      if (this.inPoint != this.outPoint)
      {
         cropData(this.inPoint, this.outPoint);
      }
      else
      {
         cropData(this.inPoint, this.inPoint + 1);
      }
   }

   public void cropData(int start, int end)
   {
      // Abort if the start or end point is unreasonable
      if ((start < 0) || (end > bufferSize))
      {
         return;    // -1; //SimulationConstructionSet.NUM_POINTS;
      }

      if (entries.isEmpty())
      {
         bufferSize = DataBufferEntry.computeBufferSizeAfterCrop(start, end, bufferSize);
      }

      // Step through the entries cropping and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         int retSize = entry.cropData(start, end);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      // Move the current index to its relative position after the resize
      this.index = ((this.index - start + bufferSize) % bufferSize);

      // If the index is out of bounds move it to the beginning
      if (this.index < 0)
      {
         this.index = 0;
      }

      if (this.index >= bufferSize)
      {
         this.index = 0;
      }

      // Set the in point to the beginning and the out point to the end
      this.inPoint = 0;
      this.outPoint = bufferSize - 1;

      // Move to the first tick
      this.tick(0);

      // +++++this.updateUI();
   }


   public void cutData()
   {
      if (this.inPoint <= this.outPoint)
      {
         cutData(this.inPoint, this.outPoint);
      }
   }

   public void cutData(int start, int end)
   {
      // Abort if the start or end point is unreasonable
      if ((start < 0) || (end > bufferSize))
      {
         return;
      }

      if (entries.isEmpty())
      {
         bufferSize = DataBufferEntry.computeBufferSizeAfterCut(start, end, bufferSize);
      }

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         int retSize = entry.cutData(start, end);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      // Move the current index to its relative position after the resize
      this.index = ((this.index - start + bufferSize) % bufferSize);

      // If the index is out of bounds move it to the beginning
      if (this.index < 0)
      {
         this.index = 0;
      }

      if (this.index >= bufferSize)
      {
         this.index = 0;
      }

      // Set the in point to the beginning and the out point to the end
      this.inPoint = 0;
      this.outPoint = start - 1;

      // Move to the first tick
      this.gotoOutPoint();
   }

   public void thinData(int keepEveryNthPoint)
   {
      packData();

      this.inPoint = 0;
      this.index = 0;

      if (bufferSize <= 2 * keepEveryNthPoint)
         return;

      // Step through the entries cutting and resizing the data set for each
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         int retSize = entry.thinData(keepEveryNthPoint);

         // If the result is a positive number store the new buffer size otherwise keep the original size
         if (retSize >= 0)
         {
            bufferSize = retSize;
         }
      }

      this.outPoint = bufferSize - 1;

      this.gotoInPoint();
   }

   public double computeAverage(YoVariable<?> variable)
   {
      DataBufferEntry entry = this.getEntry(variable);
      return entry.computeAverage();
   }
   
   @Override
   public int getInPoint()
   {
      return this.inPoint;
   }

   @Override
   public int getOutPoint()
   {
      return this.outPoint;
   }

   public void setInPoint()
   {
      setInPoint(this.index);
   }

   public void setOutPoint()
   {
      setOutPoint(this.index);
   }

   public void setInPoint(int in)
   {
      this.inPoint = in;
      keyPoints.trim(inPoint, outPoint);
   }

   public void setOutPoint(int out)
   {
      this.outPoint = out;
      keyPoints.trim(inPoint, outPoint);
   }

   public void setInOutPointFullBuffer()
   {
      this.inPoint = 0;
      this.outPoint = entries.get(0).getDataLength() - 1;
   }

   @Override
   public void gotoInPoint()
   {
      setIndex(this.inPoint);
   }

   @Override
   public void gotoOutPoint()
   {
      setIndex(this.outPoint);
   }

   public boolean atInPoint()
   {
      return index == inPoint;
   }

   public boolean atOutPoint()
   {
      return index == outPoint;
   }

   public void setKeyPoint()
   {
      keyPoints.setKeyPoint(this.index);
   }

   /**
    * Gets the KeyPoints in the cropped data
    *
    * @return The current KeyPoints as an ArrayList of Integer
    */
   public ArrayList<Integer> getKeyPoints()
   {
      // only return point in the cropped data
      return keyPoints.getPoints();
   }

   @Override
   public void setIndex(int index)
   {
      setIndex(index, true);
   }

   @Override
   public void setIndexButDoNotNotifySimulationRewoundListeners(int index)
   {
      this.setIndex(index, false);
   }

   private void setIndex(int index, boolean notifySimulationRewoundListeners)
   {
      if (safeToManualyChangeIndex)
      {
         this.index = index;

         // if (this.index > this.getMaxIndex()) this.index = 0;
         if (this.index >= bufferSize)
         {
            this.index = 0;
         }
         else if (this.index < 0)
         {
            this.index = bufferSize - 1;    // )0;
         }

         setYoVariableValuesToDataAtIndex();

         notifyIndexChangedListeners();

         // @todo: JEP 100514: Note that notifying the simulationRewoundListeners will happen in the GUI thread, not the simulation/control thread.
         // So there may be thread timing issues here. We may need to do some sort of synchronization and/or change it so that the
         // simulationRewoundListeners are notified in the simulation/control thread.
         if (notifySimulationRewoundListeners)
            notifySimulationRewoundListenerListeners();
      }
   }

   public void attachSimulationRewoundListeners(ArrayList<RewoundListener> simulationRewoundListeners)
   {
      for (RewoundListener simulationRewoundListener : simulationRewoundListeners)
      {
         attachSimulationRewoundListener(simulationRewoundListener);
      }
   }

   public void attachSimulationRewoundListener(RewoundListener simulationRewoundListener)
   {
      if (simulationRewoundListeners == null)
      {
         simulationRewoundListeners = new ArrayList<RewoundListener>();
      }

      simulationRewoundListeners.add(simulationRewoundListener);
   }

   public void attachIndexChangedListener(IndexChangedListener indexChangedListener)
   {
      if (indexChangedListeners == null)
      {
         indexChangedListeners = new ArrayList<IndexChangedListener>();
      }

      indexChangedListeners.add(indexChangedListener);
   }

   @Override
   public int getIndex()
   {
      return this.index;
   }

   @Override
   public boolean tick(int ticks)
   {
      return tick(ticks, true);
   }

   @Override
   public boolean tickButDoNotNotifySimulationRewoundListeners(int ticks)
   {
      return tick(ticks, false);
   }


   /**
    * This method attempts to step the index n points.  If the offset is within the valid data set the function returns false and the index is set to index+n.  Otherwise the index is forced to the inPoint or the outPoint depending on which is more appropriate.
    *
    * @param n Number of steps to shift the index, this value can be negative.
    * @return Indicates whether or not the index was forced to one of the ends.
    */
   private boolean tick(int n, boolean notifySimulationRewoundListeners)
   {
      if (safeToManualyChangeIndex)
      {
         int newIndex = this.index + n;

         boolean rolledOver = !isIndexBetweenInAndOutPoint(newIndex);
         if (rolledOver)
         {
            if (n >= 0)
            {
               newIndex = this.inPoint;
            }
            else
            {
               newIndex = this.outPoint;
            }
         }

         setIndex(newIndex, notifySimulationRewoundListeners);

         return rolledOver;
      }

      return false;
   }


   public boolean updateAndTick()
   {
      setDataAtIndexToYoVariableValues();
      boolean ret = tick(1);
      setYoVariableValuesToDataAtIndex();

      return ret;
   }

   public boolean updateAndTickBackwards()
   {
      setDataAtIndexToYoVariableValues();
      boolean ret = tick(-1);
      setYoVariableValuesToDataAtIndex();

      return ret;
   }

   private void setYoVariableValuesToDataAtIndex()
   {
      //noinspection ForLoopReplaceableByForEach (iterators use memory, runs in tight loop)
      for (int j = 0; j < entries.size(); j++)
      {
         DataBufferEntry entry = entries.get(j);
         entry.setYoVariableValueToDataAtIndex(this.index);
      }
   }

   public void setDataAtIndexToYoVariableValues()
   {
      //noinspection ForLoopReplaceableByForEach (iterators use memory, runs in tight loop)
      for (int j = 0; j < entries.size(); j++)
      {
         DataBufferEntry entry = entries.get(j);
         entry.setDataAtIndexToYoVariableValue(this.index);
      }
   }

   public void tickAndUpdate()
   {
      if (!clearing)
      {
         this.index = this.index + 1;

         if (this.index >= bufferSize)
         {
            if (wrapBuffer || (bufferSize >= maxBufferSize))
            {
               this.index = 0;
            }
            else    // Expand the buffer, it just overflowed and there's room to grow...
            {
               int newSize = bufferSize * 3 / 2;

               if (newSize > maxBufferSize)
               {
                  newSize = maxBufferSize;
               }

               this.enlargeBufferSize(newSize);
            }
         }

         if (this.index < 0)
         {
            this.index = 0;
         }

         // Out point should always be the last recorded tick...
         this.outPoint = this.index;

         if (this.outPoint == this.inPoint)
         {
            this.inPoint = this.inPoint + 1;

            if (this.inPoint >= bufferSize)
            {
               this.inPoint = 0;
            }
         }

         keyPoints.removeKeyPoint(index);
         setDataAtIndexToYoVariableValues();
         notifyIndexChangedListeners();
      }
      else
      {
         clearing = false;
         resetDataBuffer();
      }
   }

   protected void notifySimulationRewoundListenerListeners()
   {
      if (simulationRewoundListeners != null)
      {
         for (int i = 0; i < simulationRewoundListeners.size(); i++)
         {
            RewoundListener simulationRewoundListener = simulationRewoundListeners.get(i);

            simulationRewoundListener.wasRewound();
         }
      }

      notifyDataBufferListeners();
   }

   private void notifyIndexChangedListeners()
   {
      if (indexChangedListeners != null)
      {
         for (int i = 0; i < indexChangedListeners.size(); i++)
         {
            IndexChangedListener indexChangedListener = indexChangedListeners.get(i);

            if (t != null)
            {
               indexChangedListener.indexChanged(index, t.getDoubleValue());
            }
            else
            {
               indexChangedListener.indexChanged(index, 0.0);
            }
         }
      }

      notifyDataBufferListeners();
   }

   private void notifyDataBufferListeners()
   {
      for (int i = 0; i < dataBufferListeners.size(); i++)
      {
         DataBufferListener dataBufferListener = dataBufferListeners.get(i);
         YoVariable<?>[] yoVariables = dataBufferListener.getVariablesOfInterest(this);
         double[] values = new double[yoVariables.length];

         for (int j = 0; j < yoVariables.length; j++)
         {
            values[j] = yoVariables[j].getValueAsDouble();
         }

         dataBufferListener.dataBufferUpdate(values);
      }
   }

   public void applyDataProcessingFunction(DataProcessingFunction dataProcessingFunction)
   {
      dataProcessingFunction.initializeProcessing();
      gotoInPoint();

      while (!atOutPoint())
      {
         dataProcessingFunction.processData();
         updateAndTick();
      }

      dataProcessingFunction.processData();
      updateAndTick();
   }

   public void applyDataProcessingFunctionBackward(DataProcessingFunction dataProcessingFunction)
   {
      dataProcessingFunction.initializeProcessing();
      gotoOutPoint();

      while (!atInPoint())
      {
         dataProcessingFunction.processData();
         updateAndTickBackwards();
      }

      dataProcessingFunction.processData();
      updateAndTickBackwards();
   }

   @Override
   public boolean isKeyPointModeToggled()
   {
      return keyPoints.useKeyPoints();
   }

   @Override
   public void toggleKeyPointMode()
   {
      if (keyPoints.useKeyPoints())
      {
         keyPoints.setUseKeyPoints(false);
      }
      else
      {
         keyPoints.setUseKeyPoints(true);
      }

      for (ToggleKeyPointModeCommandListener commandListener : toggleKeyPointModeCommandListeners)
      {
         commandListener.updateKeyPointModeStatus();
      }
   }

   @Override
   public void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener commandListener)
   {
      toggleKeyPointModeCommandListeners.add(commandListener);
   }

   public void toggleCameraKeyPoint(TrackingDollyCameraController camera)
   {
      camera.setUseCameraKeyPoints(!camera.useKeyCameraPoints());
   }

   public int getNextTime()
   {
      return keyPoints.getNextTime(index, inPoint, outPoint);
   }

   public int getPreviousTime()
   {
      return keyPoints.getPreviousTime(index, inPoint, outPoint);
   }

   public ArrayList<YoVariable<?>> getVariablesThatStartWith(String searchString, boolean caseSensitive)
   {
      ArrayList<YoVariable<?>> ret = null;

      if (!caseSensitive)
      {
         searchString = searchString.toLowerCase();
      }

      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);
         String name = entry.getVariable().getName();

         if (!caseSensitive)
         {
            name = name.toLowerCase();
         }

         if (name.startsWith(searchString))
         {
            if (ret == null)
            {
               ret = new ArrayList<YoVariable<?>>();
            }

            ret.add(entry.getVariable());
         }
      }

      return ret;
   }

   public class RepeatDataBufferEntryException extends Exception
   {
      private static final long serialVersionUID = -8151095313463159788L;

      RepeatDataBufferEntryException(String message)
      {
         super(message);
      }
   }


   public ArrayList<YoVariable<?>> search(String searchText)
   {
      ArrayList<YoVariable<?>> allVariables = getAllVariables();
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      for (int i = 0; i < allVariables.size(); i++)
      {
         DataBufferEntry entry = entries.get(i);

         boolean match = RegularExpression.check(entry.getVariable().getName(), searchText);

         if (match)
         {
            ret.add(entry.getVariable());
         }
      }

      return ret;
   }

   public boolean checkIfDataIsEqual(DataBuffer dataBuffer, double epsilon)
   {
      ArrayList<DataBufferEntry> thisEntries = this.entries;
      ArrayList<DataBufferEntry> entries = dataBuffer.entries;

      if (thisEntries.size() != entries.size())
      {
         System.out.println("Sizes don't match! thisEntries.size() = " + thisEntries.size() + ", entries.size() = " + entries.size());

         return false;
      }

      for (DataBufferEntry entry : entries)
      {
         YoVariable<?> variable = entry.getVariable();
         DataBufferEntry entry2 = this.getEntry(variable.getName());

         if (entry2 == null)
         {
            System.out.println("Dont' have the same variables! Can't find " + variable.getName());

            return false;
         }

         if (!entry.checkIfDataIsEqual(entry2, this.inPoint, this.outPoint, epsilon))
         {
            System.out.println("Data in entries are different!");

            return false;
         }
      }

      return true;
   }

   public String getTimeVariableName()
   {
      return timeVariableName;
   }

   public void setTimeVariableName(String timeVariableName)
   {
      if (getEntry(timeVariableName) == null)
      {
         System.err.println("The requested timeVariableName does not exist, change not successful");
      }
      else
      {
         this.timeVariableName = timeVariableName;
      }
   }

   @Override
   public double[] getTimeData()
   {
      return getEntry(timeVariableName).getData();
   }

   @Override
   public boolean isIndexBetweenInAndOutPoint(int indexToCheck)
   {
      if (this.inPoint <= this.outPoint)
      {
         if ((indexToCheck >= this.inPoint) && (indexToCheck <= this.outPoint))
         {
            return true;
         }
      }
      else
      {
         if ((indexToCheck <= this.outPoint) || (indexToCheck > this.inPoint))
         {
            return true;
         }
      }

      return false;
   }



}
