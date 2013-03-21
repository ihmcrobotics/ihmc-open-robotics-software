package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import java.util.HashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.stateEstimation.ProcessModelElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MathTools;

public abstract class AbstractProcessModelElement implements ProcessModelElement
{
   protected final HashMap<ControlFlowOutputPort<?>, DenseMatrix64F> stateMatrixBlocks;
   private final DenseMatrix64F processNoiseCovarianceBlock;
   protected final HashMap<ControlFlowInputPort<?>, DenseMatrix64F> inputMatrixBlocks;

   public AbstractProcessModelElement(int covarianceMatrixSize, int nStateMatrixBlocks, int nInputMatrixBlocks)
   {
      processNoiseCovarianceBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
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
      return processNoiseCovarianceBlock;
   }

   public void setNoiseStandardDeviation(double standardDeviation)
   {
      double variance = MathTools.square(standardDeviation);
      CommonOps.setIdentity(processNoiseCovarianceBlock);
      CommonOps.scale(variance, processNoiseCovarianceBlock);
   }
}
