package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.commons.collections.CollectionUtils;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearDynamicSystems.SplitUpMatrixExponentialStateSpaceSystemDiscretizer;
import us.ihmc.robotics.linearDynamicSystems.StateSpaceSystemDiscretizer;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.ProcessModelElement;

public class ProcessModelElementGroup
{
   private final Map<ControlFlowOutputPort<?>, ProcessModelElement> stateToProcessModelElementMap = new LinkedHashMap<ControlFlowOutputPort<?>, ProcessModelElement>();
   private final ArrayList<ControlFlowInputPort<?>> allInputs;
   private final Map<ControlFlowInputPort<?>, Integer> processInputStartIndices = new LinkedHashMap<ControlFlowInputPort<?>, Integer>();

   private final List<ControlFlowOutputPort<?>> allStates = new ArrayList<ControlFlowOutputPort<?>>();
   private final Map<ControlFlowOutputPort<?>, Integer> allStateStartIndices = new LinkedHashMap<ControlFlowOutputPort<?>, Integer>();
   private final Map<ControlFlowOutputPort<?>, Integer> stateSizes = new LinkedHashMap<ControlFlowOutputPort<?>, Integer>();
   private final Map<ControlFlowInputPort<?>, Integer> inputSizes = new LinkedHashMap<ControlFlowInputPort<?>, Integer>();
   private final int inputMatrixSize;
   private final int stateMatrixSize;

   // continuous time process model (requires discretization)
   private final ArrayList<ProcessModelElement> continuousTimeProcessModelElements = new ArrayList<ProcessModelElement>();
   private final ArrayList<ControlFlowOutputPort<?>> continuousTimeStates;
   private final Map<ControlFlowOutputPort<?>, Integer> continuousStateStartIndices = new LinkedHashMap<ControlFlowOutputPort<?>, Integer>();
   private final boolean isContinuousTimeModelTimeVariant;
   private final StateSpaceSystemDiscretizer discretizer;
   private final DenseMatrix64F FContinuous;
   private final DenseMatrix64F GContinuous;
   private final DenseMatrix64F QContinuous;

   // discrete time process model (does not require discretization)
   private final List<ProcessModelElement> discreteTimeProcessModelElements = new ArrayList<ProcessModelElement>();
   private final ArrayList<ControlFlowOutputPort<?>> discreteTimeStates;
   private final Map<ControlFlowOutputPort<?>, Integer> discreteStateStartIndices = new LinkedHashMap<ControlFlowOutputPort<?>, Integer>();
   private final boolean isDiscreteTimeModelTimeVariant;
   private final DenseMatrix64F FDiscrete;
   private final DenseMatrix64F GDiscrete;
   private final DenseMatrix64F QDiscrete;

   // combined process model
   private final DenseMatrix64F F;
   private final DenseMatrix64F G;
   private final DenseMatrix64F Q;

   private final double controlDT;

   private final DenseMatrix64F correctionBlock = new DenseMatrix64F(1, 1);

   public ProcessModelElementGroup(Collection<ProcessModelElement> processModelElementCollection, double controlDT)
   {
      ArrayList<ProcessModelElement> processModelElements = sortProcessModelElements(processModelElementCollection);
      
      populateStateToProcessModelElementMap(stateToProcessModelElementMap, processModelElements);
      classifyProcessModelElements(continuousTimeProcessModelElements, discreteTimeProcessModelElements, processModelElements);
      determineSizes(stateSizes, inputSizes, stateToProcessModelElementMap);
      allInputs = determineInputList(processModelElements);
      inputMatrixSize = computeSize(inputSizes, allInputs);
      MatrixTools.computeIndicesIntoVector(allInputs, processInputStartIndices, inputSizes);

      continuousTimeStates = determineStateList(continuousTimeProcessModelElements);
      int continuousStateMatrixSize = computeSize(stateSizes, continuousTimeStates);
      FContinuous = new DenseMatrix64F(continuousStateMatrixSize, continuousStateMatrixSize);
      GContinuous = new DenseMatrix64F(continuousStateMatrixSize, inputMatrixSize);
      QContinuous = new DenseMatrix64F(continuousStateMatrixSize, continuousStateMatrixSize);
      discretizer = new SplitUpMatrixExponentialStateSpaceSystemDiscretizer(continuousStateMatrixSize, inputMatrixSize);
      MatrixTools.computeIndicesIntoVector(continuousTimeStates, continuousStateStartIndices, stateSizes);
      this.isContinuousTimeModelTimeVariant = determineTimeVariant(continuousTimeProcessModelElements);

      discreteTimeStates = determineStateList(discreteTimeProcessModelElements);
      int discreteStateMatrixSize = computeSize(stateSizes, discreteTimeStates);
      FDiscrete = new DenseMatrix64F(discreteStateMatrixSize, discreteStateMatrixSize);
      GDiscrete = new DenseMatrix64F(discreteStateMatrixSize, inputMatrixSize);
      QDiscrete = new DenseMatrix64F(discreteStateMatrixSize, discreteStateMatrixSize);
      MatrixTools.computeIndicesIntoVector(discreteTimeStates, discreteStateStartIndices, stateSizes);
      this.isDiscreteTimeModelTimeVariant = determineTimeVariant(discreteTimeProcessModelElements);

      if (CollectionUtils.intersection(continuousTimeStates, discreteTimeStates).size() > 0)
      {
         throw new RuntimeException("States are shared between continuous time model and discrete time model. This is currently not handled.");
      }
      allStates.addAll(continuousTimeStates);
      allStates.addAll(discreteTimeStates);
      MatrixTools.computeIndicesIntoVector(allStates, allStateStartIndices, stateSizes);

      stateMatrixSize = continuousStateMatrixSize + discreteStateMatrixSize;

      F = new DenseMatrix64F(stateMatrixSize, stateMatrixSize);
      G = new DenseMatrix64F(stateMatrixSize, inputMatrixSize);
      Q = new DenseMatrix64F(stateMatrixSize, stateMatrixSize);

      this.controlDT = controlDT;

      update(true);
   }

   public ArrayList<ProcessModelElement> sortProcessModelElements(Collection<ProcessModelElement> processModelElementCollection)
   {
      ArrayList<ProcessModelElement> processModelElements = new ArrayList<ProcessModelElement>();
      processModelElements.addAll(processModelElementCollection);
      
      ProcessModelElementAlphabeticalComparator nameComparator = new ProcessModelElementAlphabeticalComparator();
      Collections.sort(processModelElements, nameComparator);
      return processModelElements;
   }

   public void updateMatrixBlocks()
   {
      update(false);
   }

   public DenseMatrix64F getStateMatrixBlock()
   {
      return F;
   }

   public DenseMatrix64F getInputMatrixBlock()
   {
      return G;
   }

   public DenseMatrix64F getProcessCovarianceMatrixBlock()
   {
      return Q;
   }

   public void propagateState()
   {
      for(int i = 0; i <  continuousTimeProcessModelElements.size(); i++)
      {
         continuousTimeProcessModelElements.get(i).propagateState(controlDT);
      }
   
      for(int i = 0; i <  discreteTimeProcessModelElements.size(); i++)
      {
         discreteTimeProcessModelElements.get(i).propagateState(controlDT);
      }
   }

   public void correctState(DenseMatrix64F correction)
   {
      for (ControlFlowOutputPort<?> statePort : stateToProcessModelElementMap.keySet())
      {
         ProcessModelElement processModelElement = stateToProcessModelElementMap.get(statePort);
         MatrixTools.extractVectorBlock(correctionBlock, correction, statePort, allStateStartIndices, stateSizes);
   
         processModelElement.correctState(correctionBlock);
      }
   }

   public int getStateMatrixSize()
   {
      return stateMatrixSize;
   }

   public int getInputMatrixSize()
   {
      return inputMatrixSize;
   }

   public Map<ControlFlowOutputPort<?>, Integer> getStateStartIndices()
   {
      return allStateStartIndices;
   }

   private void update(boolean initialize)
   {
      if (initialize || isContinuousTimeModelTimeVariant)
      {
         for (ProcessModelElement processModelElement : continuousTimeProcessModelElements)
         {
            processModelElement.computeMatrixBlocks();
         }
         updateProcessModelBlock(FContinuous, GContinuous, QContinuous, continuousStateStartIndices, continuousTimeStates);
         discretizer.discretize(FContinuous, GContinuous, QContinuous, controlDT);
      }

      if (initialize || isDiscreteTimeModelTimeVariant)
      {
         for (ProcessModelElement processModelElement : discreteTimeProcessModelElements)
         {
            processModelElement.computeMatrixBlocks();
         }
         updateProcessModelBlock(FDiscrete, GDiscrete, QDiscrete, discreteStateStartIndices, discreteTimeStates);
      }

      if (initialize || isContinuousTimeModelTimeVariant || isDiscreteTimeModelTimeVariant)
      {
         assembleProcessModel(F, G, Q, FContinuous, GContinuous, QContinuous, FDiscrete, GDiscrete, QDiscrete);
      }
   }

   private void updateProcessModelBlock(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F Q, Map<ControlFlowOutputPort<?>, Integer> stateStartIndices,
         ArrayList<ControlFlowOutputPort<?>> statePorts)
   {
      F.zero();
      G.zero();
      Q.zero();

      for(int i = 0; i <  statePorts.size(); i++)
      {
         ControlFlowOutputPort<?> rowPort = statePorts.get(i);
         ProcessModelElement processModelElement = stateToProcessModelElementMap.get(rowPort);
         for(int j = 0; j <  statePorts.size(); j++)
         {
            ControlFlowOutputPort<?> columnPort = statePorts.get(j);
            DenseMatrix64F stateMatrixBlock = processModelElement.getStateMatrixBlock(columnPort);
            MatrixTools.insertMatrixBlock(F, stateMatrixBlock, rowPort, stateStartIndices, columnPort, stateStartIndices);
         }

         for(int j = 0; j <  allInputs.size(); j++)
         {
            ControlFlowInputPort<?> inputPort = allInputs.get(j);
            DenseMatrix64F inputMatrixBlock = processModelElement.getInputMatrixBlock(inputPort);
            MatrixTools.insertMatrixBlock(G, inputMatrixBlock, rowPort, stateStartIndices, inputPort, processInputStartIndices);
         }

         DenseMatrix64F processCovarianceMatrixBlock = processModelElement.getProcessCovarianceMatrixBlock();
         MatrixTools.insertMatrixBlock(Q, processCovarianceMatrixBlock, rowPort, stateStartIndices, rowPort, stateStartIndices);
      }
   }

   private static void populateStateToProcessModelElementMap(Map<ControlFlowOutputPort<?>, ProcessModelElement> stateToProcessModelElementMap,
         Collection<ProcessModelElement> processModelElements)
   {
      for (ProcessModelElement processModelElement : processModelElements)
      {
         ControlFlowOutputPort<?> outputState = processModelElement.getOutputState();
         stateToProcessModelElementMap.put(outputState, processModelElement);
      }
   }

   private static void classifyProcessModelElements(Collection<ProcessModelElement> continuousTimeProcessModelElements,
         List<ProcessModelElement> discreteTimeProcessModelElements, Collection<ProcessModelElement> processModelElements)
   {
      for (ProcessModelElement processModelElement : processModelElements)
      {
         switch (processModelElement.getTimeDomain())
         {
         case CONTINUOUS:
            continuousTimeProcessModelElements.add(processModelElement);
            break;

         case DISCRETE:
            discreteTimeProcessModelElements.add(processModelElement);
            break;

         default:
            break;
         }
      }
   }

   private static boolean determineTimeVariant(Collection<ProcessModelElement> processModelElements)
   {
      boolean isTimeVariant = false;
      for (ProcessModelElement processModelElement : processModelElements)
      {
         if (processModelElement.isTimeVariant())
            isTimeVariant = true;
      }
      return isTimeVariant;
   }

   private static void determineSizes(Map<ControlFlowOutputPort<?>, Integer> stateSizes, Map<ControlFlowInputPort<?>, Integer> inputSizes,
         Map<ControlFlowOutputPort<?>, ProcessModelElement> stateToProcessModelElementMap)
   {
      for (ControlFlowOutputPort<?> outputStatePort : stateToProcessModelElementMap.keySet())
      {
         ProcessModelElement processModelElement = stateToProcessModelElementMap.get(outputStatePort);
         for (ControlFlowOutputPort<?> inputStatePort : processModelElement.getInputStates())
         {
            DenseMatrix64F stateMatrixBlock = processModelElement.getStateMatrixBlock(inputStatePort);
            checkOrAddSize(stateSizes, inputStatePort, stateMatrixBlock.getNumCols());
            checkOrAddSize(stateSizes, outputStatePort, stateMatrixBlock.getNumRows());
         }
         for (ControlFlowInputPort<?> inputPort : processModelElement.getInputs())
         {
            DenseMatrix64F inputMatrixBlock = processModelElement.getInputMatrixBlock(inputPort);
            checkOrAddSize(inputSizes, inputPort, inputMatrixBlock.getNumCols());
            checkOrAddSize(stateSizes, outputStatePort, inputMatrixBlock.getNumRows());
         }
         DenseMatrix64F processCovarianceMatrixBlock = processModelElement.getProcessCovarianceMatrixBlock();
         checkOrAddSize(stateSizes, outputStatePort, processCovarianceMatrixBlock.getNumRows());
         checkOrAddSize(stateSizes, outputStatePort, processCovarianceMatrixBlock.getNumCols());
      }
      if (stateSizes.size() == 0)
         throw new RuntimeException();
   }

   private static <KeyType> void checkOrAddSize(Map<KeyType, Integer> sizes, KeyType key, int newSize)
   {
      Integer stateSize = sizes.get(key);
      if (stateSize == null)
      {
         sizes.put(key, newSize);
      }
      else if (stateSize != newSize)
      {
         throw new RuntimeException("State matrix size is ambiguous");
      }
   }

   private static ArrayList<ControlFlowOutputPort<?>> determineStateList(List<ProcessModelElement> processModelElements)
   {
      Set<ControlFlowOutputPort<?>> stateSet = new LinkedHashSet<ControlFlowOutputPort<?>>();

      for (ProcessModelElement processModelElement : processModelElements)
      {
         stateSet.add(processModelElement.getOutputState());
         stateSet.addAll(processModelElement.getInputStates());
      }

      return new ArrayList<ControlFlowOutputPort<?>>(stateSet);
   }

   private static ArrayList<ControlFlowInputPort<?>> determineInputList(Collection<ProcessModelElement> processModelElements)
   {
      Set<ControlFlowInputPort<?>> inputSet = new LinkedHashSet<ControlFlowInputPort<?>>();

      for (ProcessModelElement processModelElement : processModelElements)
      {
         inputSet.addAll(processModelElement.getInputs());
      }
      return new ArrayList<ControlFlowInputPort<?>>(inputSet);
   }

   private static <KeyType> int computeSize(Map<KeyType, Integer> sizes, List<KeyType> keys)
   {
      int ret = 0;
      for (KeyType key : keys)
      {
         ret += sizes.get(key);
      }
      return ret;
   }

   private static void assembleProcessModel(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F Q, DenseMatrix64F FContinuous, DenseMatrix64F GContinuous,
         DenseMatrix64F QContinuous, DenseMatrix64F FDiscrete, DenseMatrix64F GDiscrete, DenseMatrix64F QDiscrete)
   {
      F.zero();
      G.zero();
      Q.zero();

      CommonOps.insert(FContinuous, F, 0, 0);
      CommonOps.insert(FDiscrete, F, FContinuous.getNumRows(), FContinuous.getNumCols());

      CommonOps.insert(GContinuous, G, 0, 0);
      CommonOps.insert(GDiscrete, G, GContinuous.getNumRows(), 0);

      CommonOps.insert(QContinuous, Q, 0, 0);
      CommonOps.insert(QDiscrete, Q, QContinuous.getNumRows(), QContinuous.getNumCols());
   }
   
   private class ProcessModelElementAlphabeticalComparator implements Comparator<ProcessModelElement>
   {
      public int compare(ProcessModelElement o1, ProcessModelElement o2)
      {
         if (o1 == o2) return 0;
         
         int stringCompare = o1.getName().compareTo(o2.getName());
         if (stringCompare == 0) throw new RuntimeException("Two ProcessModelElements have the same name! " + o1.getName());
         
         return stringCompare;
      }
   }
   
   public String toString()
   {
      String ret = "";
      
      for (ControlFlowOutputPort<?> statePort : stateToProcessModelElementMap.keySet())
      {
         ProcessModelElement processModelElement = stateToProcessModelElementMap.get(statePort);
         
         ret = ret + processModelElement.getName() + "\n";
      }
      
      return ret;
   }
}