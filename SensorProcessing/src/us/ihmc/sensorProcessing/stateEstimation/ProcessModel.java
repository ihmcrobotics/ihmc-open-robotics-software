package us.ihmc.sensorProcessing.stateEstimation;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;

public class ProcessModel
{
   private final List<ProcessModelElementGroup> processModelElementGroups;
   private final DenseMatrix64F F;
   private final DenseMatrix64F G;
   private final DenseMatrix64F Q;
   private final Map<ControlFlowOutputPort<?>, Integer> stateStartIndices = new LinkedHashMap<ControlFlowOutputPort<?>, Integer>();
   private final DenseMatrix64F correctionBlock = new DenseMatrix64F(1, 1);
   private final int stateMatrixSize;
   private final int inputMatrixSize;

   public ProcessModel(List<ProcessModelElementGroup> processModelElementGroups)
   {
      this.processModelElementGroups = processModelElementGroups;
      int stateMatrixSize = 0;
      int inputMatrixSize = 0;
      for (ProcessModelElementGroup processModelElementGroup : processModelElementGroups)
      {
         Map<ControlFlowOutputPort<?>, Integer> groupStateStartIndices = processModelElementGroup.getStateStartIndices();
         for (ControlFlowOutputPort<?> statePort : groupStateStartIndices.keySet())
         {
            stateStartIndices.put(statePort, groupStateStartIndices.get(statePort) + stateMatrixSize);
         }

         stateMatrixSize += processModelElementGroup.getStateMatrixSize();
         inputMatrixSize += processModelElementGroup.getInputMatrixSize();
      }
      this.stateMatrixSize = stateMatrixSize;
      this.inputMatrixSize = inputMatrixSize;

      this.F = new DenseMatrix64F(stateMatrixSize, stateMatrixSize);
      this.G = new DenseMatrix64F(stateMatrixSize, inputMatrixSize);
      this.Q = new DenseMatrix64F(stateMatrixSize, stateMatrixSize);
   }

   public void updateMatrices()
   {
      // TODO: check if necessary:
      F.zero();
      G.zero();
      Q.zero();

      int stateMatrixStartIndex = 0;
      int inputMatrixStartIndex = 0;
      for (ProcessModelElementGroup processModelElementGroup : processModelElementGroups)
      {
         processModelElementGroup.updateMatrixBlocks();

         CommonOps.insert(processModelElementGroup.getStateMatrixBlock(), F, stateMatrixStartIndex, stateMatrixStartIndex);
         CommonOps.insert(processModelElementGroup.getInputMatrixBlock(), G, stateMatrixStartIndex, inputMatrixStartIndex);
         CommonOps.insert(processModelElementGroup.getProcessCovarianceMatrixBlock(), Q, stateMatrixStartIndex, stateMatrixStartIndex);

         stateMatrixStartIndex += processModelElementGroup.getStateMatrixSize();
         inputMatrixStartIndex += processModelElementGroup.getInputMatrixSize();
      }
   }

   public void propagateState()
   {
      for(int i = 0; i <  processModelElementGroups.size(); i++)
      {
         processModelElementGroups.get(i).propagateState();
      }
   }

   public void correctState(DenseMatrix64F correction)
   {
      int startIndex = 0;
      for(int i = 0; i <  processModelElementGroups.size(); i++)
      {
         int stateMatrixSize = processModelElementGroups.get(i).getStateMatrixSize();
         correctionBlock.reshape(stateMatrixSize, 1);
         CommonOps.extract(correction, startIndex, startIndex + stateMatrixSize, 0, 1, correctionBlock, 0, 0);

         processModelElementGroups.get(i).correctState(correctionBlock);
         startIndex += stateMatrixSize;
      }
   }

   public DenseMatrix64F getStateMatrix()
   {
      return F;
   }

   public DenseMatrix64F getInputMatrix()
   {
      return G;
   }

   public DenseMatrix64F getProcessCovarianceMatrix()
   {
      return Q;
   }

   public Map<ControlFlowOutputPort<?>, Integer> getStateStartIndices()
   {
      return stateStartIndices;
   }

   public int getStateMatrixSize()
   {
      return stateMatrixSize;
   }

   public int getInputMatrixSize()
   {
      return inputMatrixSize;
   }
   
   public String toString()
   {
      String ret = "";
      for (ProcessModelElementGroup processModelElementGroup : processModelElementGroups)
      {
         ret = ret + processModelElementGroup.toString();  
      }

      return ret;
   }
}
