package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import java.util.HashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.stateEstimation.ProcessModelElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MathTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public abstract class AbstractProcessModelElement implements ProcessModelElement
{
   protected final HashMap<ControlFlowOutputPort<?>, DenseMatrix64F> stateMatrixBlocks;
   protected final HashMap<ControlFlowInputPort<?>, DenseMatrix64F> inputMatrixBlocks;
   private final DenseMatrix64F processNoiseCovarianceBlock;
   private final DenseMatrix64F scaledProcessNoiseCovarianceMatrixBlock;
   private final DoubleYoVariable covarianceMatrixScaling;

   public AbstractProcessModelElement(int covarianceMatrixSize, int nStateMatrixBlocks, int nInputMatrixBlocks, String name, YoVariableRegistry registry)
   {
      stateMatrixBlocks = new HashMap<ControlFlowOutputPort<?>, DenseMatrix64F>(nStateMatrixBlocks);
      inputMatrixBlocks = new HashMap<ControlFlowInputPort<?>, DenseMatrix64F>(nInputMatrixBlocks);
      processNoiseCovarianceBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      scaledProcessNoiseCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      covarianceMatrixScaling = new DoubleYoVariable(name + "CovScaling", registry);
      covarianceMatrixScaling.set(1.0);
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
      scaledProcessNoiseCovarianceMatrixBlock.set(processNoiseCovarianceBlock);
      CommonOps.scale(covarianceMatrixScaling.getDoubleValue(), scaledProcessNoiseCovarianceMatrixBlock);

      return scaledProcessNoiseCovarianceMatrixBlock;
   }

   public void setNoiseStandardDeviation(double standardDeviation)
   {
      double variance = MathTools.square(standardDeviation);
      CommonOps.setIdentity(processNoiseCovarianceBlock);
      CommonOps.scale(variance, processNoiseCovarianceBlock);
   }
}
