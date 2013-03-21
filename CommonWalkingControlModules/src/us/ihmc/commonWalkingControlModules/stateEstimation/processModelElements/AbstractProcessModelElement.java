package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import java.util.HashMap;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.stateEstimation.ProcessModelElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;

public abstract class AbstractProcessModelElement implements ProcessModelElement
{

   protected final HashMap<ControlFlowOutputPort<?>, DenseMatrix64F> stateMatrixBlocks;
   private final DenseMatrix64F covarianceMatrix;
   protected final HashMap<ControlFlowInputPort<?>, DenseMatrix64F> inputMatrixBlocks;

   public AbstractProcessModelElement(int covarianceMatrixSize, int nStateMatrixBlocks, int nInputMatrixBlocks)
   {
      covarianceMatrix = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      stateMatrixBlocks = new HashMap<ControlFlowOutputPort<?>, DenseMatrix64F>(nStateMatrixBlocks);
      inputMatrixBlocks = new HashMap<ControlFlowInputPort<?>, DenseMatrix64F>(nInputMatrixBlocks);
   }

   public DenseMatrix64F getStateMatrixBlock(ControlFlowOutputPort<?> statePort)
   {
      return stateMatrixBlocks.get(statePort);
   }

   public DenseMatrix64F getInputMatrixBlock(ControlFlowInputPort<?> inputPort)
   {
      return inputMatrixBlocks.get(inputPort);
   }

   public DenseMatrix64F getProcessCovarianceMatrixBlock()
   {
      return covarianceMatrix;
   }

}