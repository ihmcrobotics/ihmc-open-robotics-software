package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;

public class ProcessModel
{
   private final List<ProcessModelElementGroup> processModelElementGroups;
   private final DenseMatrix64F F;
   private final DenseMatrix64F G;
   private final DenseMatrix64F Q;
   private final List<ControlFlowOutputPort<?>> states = new ArrayList<ControlFlowOutputPort<?>>();

   public ProcessModel(List<ProcessModelElementGroup> processModelElementGroups)
   {
      this.processModelElementGroups = processModelElementGroups;
      int stateMatrixSize = 0;
      int inputMatrixSize = 0;
      for (ProcessModelElementGroup processModelElementGroup : processModelElementGroups)
      {
         stateMatrixSize += processModelElementGroup.getStateMatrixSize();
         inputMatrixSize += processModelElementGroup.getInputMatrixSize();
         states.addAll(processModelElementGroup.getStates());
      }

      this.F = new DenseMatrix64F(stateMatrixSize, stateMatrixSize);
      this.G = new DenseMatrix64F(stateMatrixSize, inputMatrixSize);
      this.Q = new DenseMatrix64F(stateMatrixSize, stateMatrixSize);
   }

   public void update()
   {
      // TODO: check if necessary:
      F.zero();
      G.zero();
      Q.zero();

      int stateMatrixStartIndex = 0;
      int inputMatrixStartIndex = 0;
      for (ProcessModelElementGroup processModelElementGroup : processModelElementGroups)
      {
         processModelElementGroup.update();

         CommonOps.insert(processModelElementGroup.getStateMatrixBlock(), F, stateMatrixStartIndex, stateMatrixStartIndex);
         CommonOps.insert(processModelElementGroup.getInputMatrixBlock(), G, stateMatrixStartIndex, inputMatrixStartIndex);
         CommonOps.insert(processModelElementGroup.getProcessCovarianceMatrixBlock(), Q, stateMatrixStartIndex, stateMatrixStartIndex);

         stateMatrixStartIndex += processModelElementGroup.getStateMatrixSize();
         inputMatrixStartIndex += processModelElementGroup.getInputMatrixSize();
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
   
   public List<ControlFlowOutputPort<?>> getStates()
   {
      return states;
   }
}
